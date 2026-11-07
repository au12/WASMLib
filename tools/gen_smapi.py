#!/usr/bin/env python3
"""Generate the complete SM_API binding from upstream's headers.

Every SMAPI_EXPORT function in upstream/source/SM_API/inc/SmApi*.h becomes one JavaScript function. The headers
mark each parameter [in], [out], [in/out] or [opt] (an optional output); that decides how it crosses:

  inputs   -> JS arguments, in order (vectors as [x, y, z], arrays as JS arrays, kernel objects as handles —
              the object's address, a number; the TypeScript layer wraps those in classes)
  outputs  -> the return value: undefined (none), the value (one), or an object keyed by parameter name (several)
  in/out   -> both: an argument, and returned again (kernel objects are changed in place)

A status other than SM_SUCCESS throws an Error naming the status and the function. Overloads get a suffix from the
types that tell them apart (SmApiCreatePlane / SmApiCreatePlane_SmPlane). Writes:

  bind/gen/smapi.gen.cpp   the embind wrappers
  js/smapi.gen.d.ts        their TypeScript signatures, with upstream's parameter notes
  js/smapi.gen.json        the manifest (name, header, parameters, how each crosses)
"""
import json, os, re, sys, glob, collections

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INC = os.path.join(ROOT, "upstream", "source", "SM_API", "inc")
SMINC = os.path.join(ROOT, "upstream", "source", "SMLib", "inc")

# functions left out, with the reason (kept in the manifest)
SKIP = {
    "SmApiDraw": "debug drawing through the graphics callbacks; no display in WebAssembly",
}

# ---------------------------------------------------------------- parsing
def parse():
    funcs = []
    for h in sorted(glob.glob(os.path.join(INC, "SmApi*.h"))):
        text = open(h, encoding="utf-8", errors="replace").read()
        for m in re.finditer(r"SMAPI_EXPORT\s+([\w:<>\s\*&]+?)\s+(SmApi\w+)\s*\(", text, re.S):
            ret, name = m.group(1).strip(), m.group(2)
            # the parameter list: to the matching parenthesis, counting only code (notes may hold parentheses)
            depth, i, body = 1, m.end(), []
            while depth:
                eol = text.find("\n", i)
                line = text[i: eol if eol >= 0 else len(text)]
                code = line.split("//")[0]
                for j, ch in enumerate(code):
                    depth += ch == "("
                    depth -= ch == ")"
                    if not depth:
                        line = code[:j]
                        break
                body.append(line)
                i = eol + 1
            body = "\n".join(body)
            # the doc comment just above the declaration
            before = text[: m.start()]
            doc = ""
            dm = re.search(r"/\*+(?:(?!\*/).)*?PURPOSE:(.*?)\*/\s*$", before, re.S)
            if dm:
                doc = re.sub(r"\s+", " ", dm.group(1).split("NOTES:")[0].split("RETURNS:")[0].split("PARAMETERS")[0]).strip()
            params = []
            # one parameter per line, its note after ///<
            for line in body.split("\n"):
                code, _, note = line.partition("//")
                code = code.strip().rstrip(",").strip()
                if not code:
                    continue
                am = re.search(r"\[\s*(in\s*/\s*out|in|out|opt)\s*\]", note)
                ann = re.sub(r"\s", "", am.group(1)) if am else None
                note = re.sub(r"<br>|/<|\[\s*(in\s*/\s*out|in|out|opt)\s*\]\s*:?", "", note).strip(" /:<").strip()
                for piece in [p.strip() for p in code.split(",") if p.strip()]:
                    params.append(parse_param(piece, ann, note))
            funcs.append({"name": name, "header": os.path.basename(h), "ret": ret, "params": params, "doc": doc})
    return funcs

def parse_param(piece, ann, note):
    default = None
    if "=" in piece:
        piece, default = [x.strip() for x in piece.split("=", 1)]
    m = re.match(r"(.*?)(\w+)$", piece)
    ty, nm = m.group(1).strip(), m.group(2)
    ty = re.sub(r"\s+", " ", ty).replace(" *", "*").replace(" &", "&").replace("* &", "*&")
    ty = re.sub(r"\s*([*&])", r"\1", ty)
    return {"type": ty, "name": nm, "ann": ann, "default": default, "note": note}

# ---------------------------------------------------------------- types
OBJ = r"(?:Sm\w+)"
NUM = {"double", "int", "long", "ULONG", "SmBoolean", "unsigned long", "SmStatus", "float"}

def base(t):
    return t.replace("const ", "").strip()

def is_enum(t):
    return t in ENUMS

def classify(p):
    """how one parameter crosses: (direction, kind)"""
    t, a = p["type"], p["ann"]
    b = base(t)
    out_ref = b.endswith("&") and not t.startswith("const ")
    # unannotated non-const references to a pointer or an array are outputs (SmApiTessellateBoundaries)
    if a in ("out", "opt") or (a is None and out_ref and (b.endswith("*&") or b.startswith("SmTArray<"))):
        d = "out"
    elif a == "in/out":
        d = "inout"
    else:
        d = "in"
    # kinds
    core = b.rstrip("&").rstrip("*").strip()
    if core.startswith("SmTArray<"):
        # an array keeps the const of its elements (SmTArray<const SmFace*> is its own type)
        core = re.sub(r"^const\s+", "", t.strip()).rstrip("&").rstrip("*").strip()
    ptrs = re.sub(r"<.*>", "", b).rstrip("&").count("*")
    if core in NUM or is_enum(core):
        k = "num"
    elif core in ("SmVector3d", "SmPoint3d"):
        k = "v3"
    elif core in ("SmVector2d", "SmPoint2d"):
        k = "v2"
    elif core == "TCHAR":
        k = "str"
    elif core.startswith("SmTArray<"):
        k = "arr"
    elif core in ("SmTessellationParams", "SmSweepOptions", "SmAxis2Placement", "SmTessellationReport"):
        k = "struct"
    elif re.fullmatch(OBJ, core):
        k = "obj"
    else:
        k = "unknown"
    return d, k, core, ptrs

def elem(core):
    return re.fullmatch(r"SmTArray<\s*(.*?)\s*>", core).group(1)

def ts_of(core, ptrs):
    if core in NUM or is_enum(core):
        return "number"
    if core in ("SmVector3d", "SmPoint3d"):
        return "V3"
    if core in ("SmVector2d", "SmPoint2d"):
        return "V2"
    if core == "TCHAR":
        return "string"
    if core.startswith("SmTArray<"):
        e = elem(core).replace("const ", "").strip()
        ep = e.count("*")
        return ts_of(e.rstrip("*").strip(), ep) + "[]"
    if core == "SmTessellationParams":
        return "TessellationParams"
    if core == "SmSweepOptions":
        return "SweepOptions"
    if core == "SmAxis2Placement":
        return "Placement"
    if core == "SmTessellationReport":
        return "TessellationReport"
    return f"Ptr<'{core}'>"

ENUMS = set()

def find_enums():
    for h in glob.glob(os.path.join(SMINC, "*.h")) + glob.glob(os.path.join(INC, "*.h")):
        t = open(h, encoding="utf-8", errors="replace").read()
        for m in re.finditer(r"\benum\s+(?:class\s+)?(\w+)\s*(?::\s*\w+\s*)?\{", t):
            ENUMS.add(m.group(1))
        for m in re.finditer(r"typedef\s+enum\s*\w*\s*\{[^}]*\}\s*(\w+)\s*;", t, re.S):
            ENUMS.add(m.group(1))

def status_codes():
    codes = {}
    for h in sorted(glob.glob(os.path.join(SMINC, "*.h"))):
        for m in re.finditer(r"^\s*#define\s+(SM_(?:SUCCESS|ERR\w*|WARN\w*))\s+(-?\d+)L?\b", open(h, encoding="utf-8", errors="replace").read(), re.M):
            codes.setdefault(int(m.group(2)), m.group(1))
    return codes

# ---------------------------------------------------------------- C++ generation
def cpp_in(p, i, d, k, core, ptrs):
    """declare the C++ argument for an input (or in/out) parameter from JS value a<i>; returns (decls, expr)"""
    t = p["type"]
    a = f"a{i}"
    opt = p["default"] is not None
    if k == "num":
        cast = core if core not in ("SmBoolean",) else "int"
        if b_is_ref(t) and not t.startswith("const "):
            return [f"{cast} v{i} = ({cast}){a}.as<double>();"], f"v{i}"
        return [], f"({cast}){a}.as<double>()"
    if k == "v3":
        decl = [f"SmVector3d v{i} = wl::v3({a});"]
        if ptrs:  # const SmVector3d* optional
            return [f"SmVector3d v{i}; const bool h{i} = wl::present({a}); if (h{i}) v{i} = wl::v3({a});"], f"(h{i} ? &v{i} : nullptr)"
        return decl, f"v{i}"
    if k == "v2":
        if ptrs:
            return [f"SmVector2d v{i}; const bool h{i} = wl::present({a}); if (h{i}) v{i} = wl::v2({a});"], f"(h{i} ? &v{i} : nullptr)"
        return [f"SmVector2d v{i} = wl::v2({a});"], f"v{i}"
    if k == "str":
        return [f"std::string s{i} = {a}.as<std::string>();"], f"s{i}.c_str()"
    if k == "obj":
        star = "*" * max(1, ptrs if not t.endswith("&") else ptrs)
        cty = core + "*"
        if base(t).endswith("*&"):  # in/out pointer reference
            return [f"{cty} p{i} = wl::ptr<{core}>({a});"], f"p{i}"
        if ptrs == 0:  # by reference to an object
            return [], f"*wl::ptr<{core}>({a})"
        return [], f"wl::ptr<{core}>({a})"
    if k == "arr":
        e = elem(core)
        if ptrs:  # SmTArray<T>* in
            return [f"{core} v{i}; const bool h{i} = wl::present({a}); if (h{i}) wl::fill({a}, v{i});"], f"(h{i} ? &v{i} : nullptr)"
        return [f"{core} v{i}; wl::fill({a}, v{i});"], f"v{i}"
    if k == "struct":
        if ptrs:
            return [f"{core} v{i}; const bool h{i} = wl::present({a}); if (h{i}) wl::from({a}, v{i});"], f"(h{i} ? &v{i} : nullptr)"
        return [f"{core} v{i}; if (wl::present({a})) wl::from({a}, v{i});"], f"v{i}"
    raise ValueError(f"no input mapping for {t}")

def b_is_ref(t):
    return base(t).endswith("&")

def cpp_out(p, i, d, k, core, ptrs):
    """declare storage for an output parameter; returns (decls, expr, result-conversion)"""
    t = base(p["type"])
    if k == "num":
        cast = "int" if core == "SmBoolean" else core
        return [f"{cast} o{i}{{}};"], (f"&o{i}" if t.endswith("*") else f"o{i}"), f"wl::num(o{i})"
    if k == "v3":
        if t.endswith("*&"):  # SmVector3d*& — SmApiEvaluateCurve writes through the pointer when it isn't null
            return [f"SmVector3d s{i}; SmVector3d* o{i} = &s{i};"], f"o{i}", f"wl::js(s{i})"
        return [f"SmVector3d o{i};"], f"o{i}", f"wl::js(o{i})"
    if k == "v2":
        return [f"SmVector2d o{i};"], f"o{i}", f"wl::js(o{i})"
    if k == "obj":
        return [f"{core}* o{i} = nullptr;"], f"o{i}", f"wl::handle(o{i})"
    if k == "arr":
        return [f"{core} o{i};"], (f"&o{i}" if t.endswith("*") else f"o{i}"), f"wl::js(o{i})"
    if k == "struct":
        return [f"{core} o{i};"], (f"&o{i}" if t.endswith("*") else f"o{i}"), f"wl::js(o{i})"
    raise ValueError(f"no output mapping for {p['type']}")

def generate(funcs):
    find_enums()
    by = collections.defaultdict(list)
    for f in funcs:
        by[f["name"]].append(f)
    cpp, dts, manifest, skipped = [], [], [], []
    for name, group in by.items():
        if name in SKIP:
            for f in group:
                skipped.append({"name": name, "header": f["header"], "why": SKIP[name]})
            continue
        # overload suffixes: from the types that differ
        sigs = [[p["type"] for p in f["params"]] for f in group]
        for gi, f in enumerate(group):
            js = name
            if len(group) > 1:
                others = [s for j, s in enumerate(sigs) if j != gi]
                mine = sigs[gi]
                diff = [base(t).rstrip("*&").replace("SmTArray<", "").replace(">", "").replace("*", "") for k, t in enumerate(mine)
                        if all(k >= len(o) or o[k] != t for o in others)]
                suffix = "_".join(dict.fromkeys(d.replace("Sm", "", 1) if d.startswith("Sm") else d for d in diff)) or f"{len(mine)}"
                js = name if gi == 0 else f"{name}_{suffix}"
            try:
                cpp.append(gen_one(f, js))
            except ValueError as e:
                skipped.append({"name": js, "header": f["header"], "why": str(e)})
                continue
            dts.append(gen_dts(f, js))
            manifest.append({"js": js, "name": name, "header": f["header"], "doc": f["doc"],
                             "params": [{**p, "cross": classify(p)[0], "kind": classify(p)[1]} for p in f["params"]]})
    return cpp, dts, manifest, skipped

def gen_one(f, js):
    lines, call, outs, ins = [], [], [], 0
    sig = []
    for i, p in enumerate(f["params"]):
        d, k, core, ptrs = classify(p)
        if k == "unknown":
            raise ValueError(f"parameter type {p['type']} has no mapping")
        if d == "out":
            decl, expr, conv = cpp_out(p, i, d, k, core, ptrs)
            lines += decl; call.append(expr); outs.append((p["name"], conv))
        else:
            decl, expr = cpp_in(p, i, d, k, core, ptrs)
            sig.append(f"const val& a{i}")
            lines += decl; call.append(expr)
            if d == "inout" and k in ("arr",):
                outs.append((p["name"], f"wl::js(v{i})"))
            elif d == "inout" and k == "num" and b_is_ref(p["type"]):
                outs.append((p["name"], f"wl::num(v{i})"))
            elif d == "inout" and k == "obj" and base(p["type"]).endswith("*&"):
                outs.append((p["name"], f"wl::handle(p{i})"))
    body = "\n    ".join(lines)
    callexpr = f"{f['name']}({', '.join(call)})"
    if f["ret"] == "SmApiStatus":
        run = f"wl::check({callexpr}, \"{f['name']}\");"
    elif f["ret"] == "void":
        run = f"{callexpr};"
    else:
        run = f"auto r = {callexpr};"
        outs.insert(0, ("result", "wl::handle(r)" if "*" in f["ret"] else "wl::num(r)"))
    if not outs:
        ret = "return val::undefined();"
    elif len(outs) == 1:
        ret = f"return {outs[0][1]};"
    else:
        ret = "val o = val::object();\n    " + "\n    ".join(f'o.set("{n}", {c});' for n, c in outs) + "\n    return o;"
    # the kernel's own C++ exceptions and a failed status both become one JavaScript Error, thrown after the try block
    # (a JavaScript exception thrown inside it could be caught by catch (...) under native WebAssembly exceptions)
    ind = lambda t: t.replace("\n    ", "\n        ")
    return f"""// {f['header']}
static val w_{js}({', '.join(sig)}) {{
    std::string err;
    try {{
        {ind(body)}
        {run}
        {ind(ret)}
    }} catch (const std::exception& e) {{ err = e.what(); }} catch (...) {{ err = "unknown C++ exception"; }}
    wl::raise("{f['name']}", err);
}}""", js

def gen_dts(f, js):
    args, outs = [], []
    for i, p in enumerate(f["params"]):
        d, k, core, ptrs = classify(p)
        ty = ts_of(core, ptrs)
        if d != "out":
            optional = p["default"] is not None or (ptrs and k in ("v3", "v2", "arr", "struct"))
            args.append(f"{p['name']}{'?' if optional else ''}: {ty}{' | null' if optional else ''}")
        if d == "out" or (d == "inout" and (k == "arr" or (k == "num" and b_is_ref(p["type"])) or (k == "obj" and base(p["type"]).endswith("*&")))):
            outs.append((p["name"], ty))
    if f["ret"] not in ("SmApiStatus", "void"):
        outs.insert(0, ("result", "number" if "*" not in f["ret"] else f"Ptr<'{f['ret'].rstrip('*').strip()}'>"))
    rt = "void" if not outs else outs[0][1] if len(outs) == 1 else "{ " + "; ".join(f"{n}: {t}" for n, t in outs) + " }"
    notes = "\n".join(f"   * @param {p['name']} {p['note']}" for p in f["params"] if p["note"] and classify(p)[0] != "out")
    outnotes = "\n".join(f"   *   - {p['name']}: {p['note']}" for p in f["params"] if p["note"] and classify(p)[0] == "out")
    doc = f"  /** {f['doc'] or f['name']}" + (f"\n{notes}" if notes else "") + (f"\n   * @returns\n{outnotes}" if outnotes else "") + "\n   */"
    return f"{doc}\n  {js}({', '.join(args)}): {rt};"

def main():
    funcs = parse()
    cpp, dts, manifest, skipped = generate(funcs)
    codes = status_codes()
    os.makedirs(os.path.join(ROOT, "bind", "gen"), exist_ok=True)
    os.makedirs(os.path.join(ROOT, "js"), exist_ok=True)
    with open(os.path.join(ROOT, "bind", "gen", "smapi.gen.cpp"), "w") as o:
        o.write("// GENERATED by tools/gen_smapi.py from upstream's SM_API headers — do not edit.\n")
        o.write('#include "wl.h"\n\n')
        o.write("\n\n".join(c for c, _ in cpp))
        o.write("\n\nEMSCRIPTEN_BINDINGS(smapi_generated) {\n")
        for _, js in cpp:
            o.write(f'    function("{js}", &w_{js});\n')
        o.write("}\n\n// status codes, by value\nconst char* wl::status_name(long s) {\n    switch (s) {\n")
        for v, n in sorted(codes.items()):
            o.write(f'        case {v}: return "{n}";\n')
        o.write('        default: return "SM_STATUS_UNKNOWN";\n    }\n}\n')
    with open(os.path.join(ROOT, "js", "smapi.gen.d.ts"), "w") as o:
        o.write("// GENERATED by tools/gen_smapi.py from upstream's SM_API headers — do not edit.\n")
        o.write("// Parameter notes are upstream's (usd-brep, Apache-2.0).\n")
        o.write("import type { V2, V3, Ptr, TessellationParams, SweepOptions, Placement, TessellationReport } from './types';\n\n")
        o.write("/** Every function of SMLib's SM_API, one to one. */\nexport interface SmApi {\n")
        o.write("\n".join(dts))
        o.write("\n}\n\n/** status codes by name */\nexport declare const STATUS: {\n")
        for v, n in sorted(codes.items()):
            o.write(f"  {n}: {v};\n")
        o.write("};\n")
    with open(os.path.join(ROOT, "js", "smapi.gen.json"), "w") as o:
        json.dump({"functions": manifest, "skipped": skipped, "status": {n: v for v, n in sorted(codes.items())}}, o, indent=1)
    print(f"{len(cpp)} functions bound, {len(skipped)} left out", file=sys.stderr)
    for s in skipped:
        print(f"  - {s['name']}: {s['why']}", file=sys.stderr)

if __name__ == "__main__":
    main()

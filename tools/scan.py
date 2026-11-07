import re, glob, collections, sys
src = sys.argv[1]
funcs = []
for h in sorted(glob.glob(src + "/SM_API/inc/SmApi*.h")):
    t = open(h, encoding="utf-8", errors="replace").read()
    for m in re.finditer(r"SMAPI_EXPORT\s+([\w:<>\s\*&]+?)\s+(SmApi\w+)\s*\((.*?)\)\s*;", t, re.S):
        ret, name, body = m.group(1).strip(), m.group(2), m.group(3)
        params = []
        for line in body.split("\n"):
            code = line.split("//")[0].strip().rstrip(",")
            ann = re.search(r"\[(in|out|in/out|in\s*/\s*out|in,out|opt|in ?)\s*\]", line)
            if not code: continue
            for piece in code.split(","):
                piece = piece.strip()
                if piece: params.append((piece, ann.group(1).strip() if ann else "?"))
        funcs.append((h.split("/")[-1], ret, name, params))
print(len(funcs), "functions")
types = collections.Counter()
for f in funcs:
    for p, a in f[3]:
        ty = re.sub(r"\s*=\s*.*$", "", p)
        ty = re.sub(r"\b\w+\s*(\[\])?$", "", ty).strip() if " " in ty or "*" in ty or "&" in ty else ty
        types[(ty, a)] += 1
for (t, a), n in types.most_common(): print(f"{n:4} {a:7} {t}")
print(collections.Counter(f[1] for f in funcs))

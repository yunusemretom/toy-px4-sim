#!/bin/bash
# AVL'yi toy.avl uzerinde alpha = 0'da calistirir ve turev dosyalarini yazar.
# Kullanim: ./run_avl.sh /yol/Avl/bin/avl
set -e
AVL=${1:-$HOME/Avl/bin/avl}
cd "$(dirname "$0")"
rm -f toy_st.txt toy_sb.txt
"$AVL" <<'EOF'
load toy.avl
oper
a a 0
x
st toy_st.txt
sb toy_sb.txt

quit
EOF
echo
echo "Yazildi: $(pwd)/toy_st.txt  $(pwd)/toy_sb.txt"
echo "Simdi: python3 avl_to_sdf.py"

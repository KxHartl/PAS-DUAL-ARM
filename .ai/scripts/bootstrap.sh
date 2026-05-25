#!/usr/bin/env bash
# Bootstrap a new LiteRealm project.
# Usage:
#   ./.ai/scripts/bootstrap.sh --name "Autonomni_Vilicari"
#   ./.ai/scripts/bootstrap.sh --name "Seminar_MEV" --rag cloud

set -e

NAME=""
RAG="none"
BRAIN="global"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --name)  NAME="$2"; shift 2 ;;
        --rag)   RAG="$2"; shift 2 ;;
        --brain) BRAIN="$2"; shift 2 ;;
        *)       echo "Unknown option: $1"; exit 1 ;;
    esac
done

if [[ -z "$NAME" ]]; then
    echo "Error: --name is required."
    echo "Usage: ./.ai/scripts/bootstrap.sh --name \"Project_Name\" [--rag none|cloud|local]"
    exit 1
fi

root="$(cd "$(dirname "$0")/../.." && pwd)"

echo ""
echo "=== LiteRealm Bootstrap ==="
echo "Projekt: $NAME"
echo "RAG:     $RAG"
echo "Brain:   $BRAIN"
echo ""

# --- 1. Set project name ---
echo "[1/5] Postavljam naziv projekta..."
sed -i "s/_TBD_/$NAME/g" "$root/STATE.md" 2>/dev/null || true
sed -i "s/\"TBD\"/\"$NAME\"/g" "$root/.ai/config/project.yaml" 2>/dev/null || true
echo "  Naziv '$NAME' upisan."

# --- 2. Create directories ---
echo "[2/5] Kreiram direktorije..."
mkdir -p "$root"/{docs,src,dist,data/raw,data/processed}
if [[ "$RAG" != "none" ]]; then
    mkdir -p "$root/data/rag/sources"
fi
echo "  Direktoriji kreirani."

# --- 3. Setup .env ---
echo "[3/5] Konfiguriram .env..."
if [[ ! -f "$root/.env" ]] && [[ -f "$root/.env.example" ]]; then
    cp "$root/.env.example" "$root/.env"
    echo "  .env kreiran."

    if [[ "$BRAIN" == "global" ]]; then
        brain_path="$HOME/.agentbrain"
        mkdir -p "$brain_path"
        echo "  Global Brain: $brain_path"
    fi
else
    echo "  .env vec postoji, preskačem."
fi

# --- 4. Python venv + dependencies ---
echo "[4/5] Python okruzenje..."
python_cmd=""
for cmd in python3 python; do
    if $cmd --version 2>&1 | grep -q "Python 3\."; then
        python_cmd="$cmd"
        break
    fi
done

if [[ -n "$python_cmd" ]]; then
    if [[ ! -d "$root/.venv" ]]; then
        $python_cmd -m venv "$root/.venv"
    fi

    pip_cmd="$root/.venv/bin/pip"
    $pip_cmd install -q python-dotenv 2>/dev/null

    if [[ "$RAG" == "cloud" ]]; then
        echo "  Instaliram RAG Cloud pakete..."
        $pip_cmd install -q langchain langchain-community chromadb pypdf google-generativeai 2>/dev/null
    elif [[ "$RAG" == "local" ]]; then
        echo "  Instaliram RAG Local pakete..."
        $pip_cmd install -q langchain langchain-community chromadb pypdf sentence-transformers 2>/dev/null
    fi
    echo "  Python OK ($python_cmd)."
else
    echo "  Python nije pronadjen — preskačem venv."
fi

# --- 5. Check LaTeX ---
echo "[5/5] Provjeravam LaTeX..."
if command -v latexmk &>/dev/null; then
    echo "  latexmk pronadjen."
else
    echo "  latexmk NIJE pronadjen — instaliraj TeX Live ili MiKTeX."
fi

echo ""
echo "=== Bootstrap zavrsen ==="
echo ""
echo "Sljedeci koraci:"
echo "  1. Popuni STATE.md s detaljima o radu"
echo "  2. Kopiraj LaTeX predlozak iz .ai/templates/ u docs/"
echo "  3. Pokreni AI agenta i reci mu sto treba napisati"
if [[ "$RAG" != "none" ]]; then
    echo "  4. Stavi PDF izvore u data/rag/sources/ za RAG citiranje"
fi
echo ""

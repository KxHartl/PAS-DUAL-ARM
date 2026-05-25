"""
LiteRealm RAG — Query pipeline with source citations.
Searches the vector store and returns answers with references.

Usage:
    python .ai/rag/query.py "Što je autonomni viličar?"
"""

import os
import sys
from pathlib import Path


def get_root():
    """Find project root (where STATE.md lives)."""
    p = Path(__file__).resolve().parent
    while p != p.parent:
        if (p / "STATE.md").exists():
            return p
        p = p.parent
    return Path.cwd()


def load_vectorstore(root):
    """Load existing ChromaDB vector store."""
    store_dir = root / "data" / "rag" / "vector_store"

    if not store_dir.exists():
        print("Vector store not found. Run ingest.py first:")
        print("  python .ai/rag/ingest.py")
        sys.exit(1)

    from dotenv import load_dotenv
    load_dotenv(root / ".env")

    # Match the embedding provider used during ingestion
    embeddings = None
    api_key = os.environ.get("GEMINI_API_KEY")

    if api_key:
        try:
            from langchain_community.embeddings import GoogleGenerativeAIEmbeddings
            embeddings = GoogleGenerativeAIEmbeddings(
                model="models/embedding-001",
                google_api_key=api_key
            )
        except Exception:
            pass

    if embeddings is None:
        try:
            from langchain_community.embeddings import HuggingFaceEmbeddings
            embeddings = HuggingFaceEmbeddings(model_name="all-MiniLM-L6-v2")
        except ImportError:
            print("No embedding provider. Set GEMINI_API_KEY or install sentence-transformers.")
            sys.exit(1)

    from langchain_community.vectorstores import Chroma
    return Chroma(
        persist_directory=str(store_dir),
        embedding_function=embeddings
    )


def query(question: str, k: int = 5) -> list[dict]:
    """
    Query the RAG vector store. Returns list of relevant chunks with metadata.
    Each result has: content, source_file, page.
    """
    root = get_root()
    vectorstore = load_vectorstore(root)

    results = vectorstore.similarity_search(question, k=k)

    output = []
    for doc in results:
        output.append({
            "content": doc.page_content,
            "source_file": doc.metadata.get("source_file", "unknown"),
            "page": doc.metadata.get("page", "?"),
        })

    return output


def main():
    if len(sys.argv) < 2:
        print("Usage: python .ai/rag/query.py \"Your question here\"")
        sys.exit(1)

    question = " ".join(sys.argv[1:])
    print(f"\nQuery: {question}\n")
    print("=" * 60)

    results = query(question)

    if not results:
        print("No relevant results found.")
        return

    for i, r in enumerate(results, 1):
        print(f"\n--- Result {i} ---")
        print(f"Source: {r['source_file']}, Page: {r['page']}")
        print(f"Content: {r['content'][:500]}...")

    # Print citation summary
    print("\n" + "=" * 60)
    print("Izvori:")
    sources = set()
    for r in results:
        sources.add(f"  [{r['source_file']}, str. {r['page']}]")
    for s in sorted(sources):
        print(s)


if __name__ == "__main__":
    main()

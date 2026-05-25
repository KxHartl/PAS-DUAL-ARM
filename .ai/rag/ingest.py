"""
LiteRealm RAG — Document ingestion pipeline.
Parses PDFs from data/rag/sources/ into a ChromaDB vector store.

Usage:
    python .ai/rag/ingest.py
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

def ingest():
    root = get_root()
    sources_dir = root / "data" / "rag" / "sources"
    store_dir = root / "data" / "rag" / "vector_store"

    if not sources_dir.exists():
        print(f"No sources directory found at {sources_dir}")
        print("Create it and add PDF files, then run again.")
        sys.exit(1)

    pdf_files = list(sources_dir.glob("*.pdf"))
    if not pdf_files:
        print(f"No PDF files found in {sources_dir}")
        sys.exit(1)

    print(f"Found {len(pdf_files)} PDF(s) in {sources_dir}")

    # Import dependencies (installed by bootstrap)
    try:
        from langchain_community.document_loaders import PyPDFLoader
        from langchain.text_splitter import RecursiveCharacterTextSplitter
        from langchain_community.vectorstores import Chroma
    except ImportError as e:
        print(f"Missing dependency: {e}")
        print("Run bootstrap with -Rag cloud or -Rag local to install RAG packages.")
        sys.exit(1)

    # Try cloud embeddings first, fall back to local
    embeddings = None
    try:
        from dotenv import load_dotenv
        load_dotenv(root / ".env")

        api_key = os.environ.get("GEMINI_API_KEY")
        if api_key:
            from langchain_community.embeddings import GoogleGenerativeAIEmbeddings
            embeddings = GoogleGenerativeAIEmbeddings(
                model="models/embedding-001",
                google_api_key=api_key
            )
            print("Using Gemini cloud embeddings.")
    except Exception:
        pass

    if embeddings is None:
        try:
            from langchain_community.embeddings import HuggingFaceEmbeddings
            embeddings = HuggingFaceEmbeddings(
                model_name="all-MiniLM-L6-v2"
            )
            print("Using local HuggingFace embeddings.")
        except ImportError:
            print("No embedding provider available.")
            print("Set GEMINI_API_KEY in .env or install sentence-transformers.")
            sys.exit(1)

    # Load and split documents
    all_docs = []
    splitter = RecursiveCharacterTextSplitter(
        chunk_size=1000,
        chunk_overlap=200,
        separators=["\n\n", "\n", ". ", " "]
    )

    for pdf_path in pdf_files:
        print(f"  Parsing: {pdf_path.name}")
        try:
            loader = PyPDFLoader(str(pdf_path))
            pages = loader.load()
            chunks = splitter.split_documents(pages)
            # Add source metadata for citations
            for chunk in chunks:
                chunk.metadata["source_file"] = pdf_path.name
            all_docs.extend(chunks)
        except Exception as e:
            print(f"  Error parsing {pdf_path.name}: {e}")

    if not all_docs:
        print("No documents were successfully parsed.")
        sys.exit(1)

    print(f"Total chunks: {len(all_docs)}")

    # Create/update vector store
    print(f"Building vector store at {store_dir}...")
    store_dir.mkdir(parents=True, exist_ok=True)

    vectorstore = Chroma.from_documents(
        documents=all_docs,
        embedding=embeddings,
        persist_directory=str(store_dir)
    )

    print(f"Done. {len(all_docs)} chunks indexed.")
    return vectorstore


if __name__ == "__main__":
    ingest()

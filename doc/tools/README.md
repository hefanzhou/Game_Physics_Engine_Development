# PDF Reader Skill

A reusable Python tool for reading and extracting structured content from PDF files.

## Dependencies

```bash
pip install PyPDF2
```

## Quick Start

### Command Line Usage

```bash
python pdf_reader.py <pdf_path> <command> [options]
```

### Python Module Usage

```python
from pdf_reader import PDFReader

reader = PDFReader("path/to/book.pdf")
```

## Commands

| Command | Description | Example |
|---------|-------------|---------|
| `info` | Show PDF metadata and basic info | `python pdf_reader.py book.pdf info` |
| `toc` | Extract table of contents | `python pdf_reader.py book.pdf toc` |
| `chapter N` | Extract chapter N content | `python pdf_reader.py book.pdf chapter 14` |
| `pages S E` | Extract pages S to E (0-indexed) | `python pdf_reader.py book.pdf pages 300 350` |
| `search TEXT` | Search for text across all pages | `python pdf_reader.py book.pdf search "collision"` |
| `index` | Build chapter-to-page index | `python pdf_reader.py book.pdf index` |

## API Reference

### `PDFReader(pdf_path)`

Initialize reader with a PDF file path.

**Attributes:**
- `reader` - PyPDF2.PdfReader instance
- `total_pages` - Total number of pages in the PDF
- `pdf_path` - Path to the PDF file

---

### `get_info() -> dict`

Returns PDF metadata including title, author, page count, and whether bookmarks exist.

```python
info = reader.get_info()
# {'file': '...', 'total_pages': 481, 'title': None, 'author': None, ...}
```

---

### `get_bookmarks(outline=None, level=0) -> list`

Recursively extract PDF bookmarks/outline as a flat list.

```python
bookmarks = reader.get_bookmarks()
# [{'title': 'Chapter 1', 'page': 12, 'level': 0}, ...]
```

---

### `extract_toc_from_text(toc_page_range=None) -> str`

Extract table of contents by scanning page text. Useful when PDF has no embedded bookmarks.

```python
# Default: scans pages 3-20
toc = reader.extract_toc_from_text()

# Custom range
toc = reader.extract_toc_from_text((5, 15))
```

---

### `build_chapter_index(chapter_pattern=None) -> list`

Scan all pages to find chapter headings and build an index.

```python
chapters = reader.build_chapter_index()
# [{'chapter_num': 1, 'title': 'Introduction', 'page': 12}, ...]

# Custom pattern (e.g., for "Part N" style headings)
chapters = reader.build_chapter_index(r'(?i)part\s+(\d+)[:\s]*([^\n]*)')
```

**Note:** Scans all pages — may be slow for large PDFs (400+ pages).

---

### `find_chapter_range(chapter_num, chapter_index=None) -> tuple | None`

Find the start and end page of a specific chapter.

```python
ch_range = reader.find_chapter_range(14)
# (310, 340) or None if not found
```

---

### `extract_pages(start_page, end_page, include_page_markers=True) -> str`

Extract text from a range of pages (0-indexed, end exclusive).

```python
text = reader.extract_pages(300, 310)
text = reader.extract_pages(300, 310, include_page_markers=False)
```

---

### `extract_chapter(chapter_num) -> str`

Extract the full text of a specific chapter. Automatically finds chapter boundaries.

```python
text = reader.extract_chapter(14)
```

Uses `build_chapter_index()` first, falls back to direct text search if not found.

---

### `search(query, case_sensitive=False) -> list`

Search for text across all pages. Returns matching pages with surrounding context.

```python
results = reader.search("collision detection")
# [{'page': 250, 'context': '...surrounding text...'}, ...]
```

## Implementation Notes

### Encoding Handling

- Windows console output is wrapped with UTF-8 encoding via `_fix_encoding()`
- This function is only called in `main()` (CLI mode), not on import
- Prevents GBK encoding errors on Windows

### Chapter Detection Strategy

1. **Primary:** Regex scan of first 500 chars of each page for "Chapter N" patterns
2. **Fallback:** Direct string search for "CHAPTER N" / "Chapter N" in first 300 chars
3. Chapter end is determined by the start of the next chapter (or end of PDF)

### Page Numbering

- All page numbers are **0-indexed** (PDF internal page numbers)
- The `include_page_markers` option adds approximate book page numbers (`page - 10` offset)
- Adjust the offset in `extract_pages()` based on your PDF's front matter

### Performance Considerations

- `search` and `index` commands scan all pages — expect ~10-30 seconds for 400+ page PDFs
- `extract_pages` and `extract_chapter` are fast once page range is known
- Consider caching `build_chapter_index()` results for repeated use

### Limitations

- Text extraction quality depends on PDF structure (scanned PDFs may yield poor results)
- For scanned/image PDFs, consider using OCR tools (e.g., `pytesseract` + `pdf2image`)
- Complex layouts (multi-column, tables) may not extract cleanly with PyPDF2
- For better extraction quality, consider `pdfplumber` as an alternative backend

## File Location

```
f:\Work\cyclone-physics\doc\tools\pdf_reader.py
```

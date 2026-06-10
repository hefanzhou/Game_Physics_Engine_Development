#!/usr/bin/env python3
"""
PDF Reader Skill - A reusable tool for reading and extracting content from PDF files.

Usage:
    python pdf_reader.py <pdf_path> [command] [options]

Commands:
    info        - Show PDF metadata and basic info
    toc         - Extract and display table of contents (bookmarks or text-based)
    chapter N   - Extract chapter N content
    pages S E   - Extract pages from S to E (0-indexed PDF page numbers)
    search TEXT - Search for text across all pages
    index       - Build a chapter-to-page index by scanning for chapter headings

Examples:
    python pdf_reader.py book.pdf info
    python pdf_reader.py book.pdf toc
    python pdf_reader.py book.pdf chapter 14
    python pdf_reader.py book.pdf pages 300 350
    python pdf_reader.py book.pdf search "collision detection"
    python pdf_reader.py book.pdf index

Notes:
    - Requires PyPDF2: pip install PyPDF2
    - For better text extraction, also install: pip install pdfplumber
    - Output is UTF-8 encoded to handle special characters
"""

import sys
import io
import re
import argparse

def _fix_encoding():
    """Fix encoding for Windows console/pipe output."""
    if sys.platform == 'win32':
        sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
        sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding='utf-8', errors='replace')

try:
    import PyPDF2
except ImportError:
    print("ERROR: PyPDF2 not installed. Run: pip install PyPDF2")
    sys.exit(1)


class PDFReader:
    """A comprehensive PDF reader for extracting structured content."""

    def __init__(self, pdf_path):
        self.pdf_path = pdf_path
        self.reader = PyPDF2.PdfReader(pdf_path)
        self.total_pages = len(self.reader.pages)

    def get_info(self):
        """Get PDF metadata and basic information."""
        meta = self.reader.metadata
        info = {
            'file': self.pdf_path,
            'total_pages': self.total_pages,
            'title': meta.title if meta else None,
            'author': meta.author if meta else None,
            'subject': meta.subject if meta else None,
            'creator': meta.creator if meta else None,
            'has_bookmarks': bool(self.reader.outline),
        }
        return info

    def get_bookmarks(self, outline=None, level=0):
        """Recursively extract PDF bookmarks/outline as a flat list."""
        if outline is None:
            outline = self.reader.outline
        if not outline:
            return []

        bookmarks = []
        for item in outline:
            if isinstance(item, list):
                # Nested bookmarks (sub-chapters)
                bookmarks.extend(self.get_bookmarks(item, level + 1))
            else:
                # A bookmark entry
                try:
                    page_num = self.reader.get_destination_page_number(item)
                except:
                    page_num = None
                bookmarks.append({
                    'title': item.title,
                    'page': page_num,
                    'level': level,
                })
        return bookmarks

    def extract_toc_from_text(self, toc_page_range=None):
        """
        Extract table of contents by scanning page text.
        Useful when PDF has no embedded bookmarks.

        Args:
            toc_page_range: tuple (start, end) of PDF pages to scan for TOC.
                           If None, scans pages 3-20.
        """
        if toc_page_range is None:
            toc_page_range = (3, min(20, self.total_pages))

        toc_text = ""
        for i in range(toc_page_range[0], toc_page_range[1]):
            toc_text += self.reader.pages[i].extract_text() + "\n"
        return toc_text

    def build_chapter_index(self, chapter_pattern=None):
        """
        Scan all pages to find chapter headings and build an index.

        Args:
            chapter_pattern: regex pattern to match chapter headings.
                           Default matches "Chapter N" or "CHAPTER N" patterns.

        Returns:
            List of dicts with 'title', 'page', 'chapter_num'
        """
        if chapter_pattern is None:
            chapter_pattern = r'(?i)chapter\s+(\d+)[:\s]*([^\n]*)'

        chapters = []
        for i in range(self.total_pages):
            text = self.reader.pages[i].extract_text()
            if not text:
                continue
            # Only check first 500 chars of each page (chapter titles are at top)
            header_text = text[:500]
            matches = re.finditer(chapter_pattern, header_text)
            for match in matches:
                chapter_num = int(match.group(1))
                title = match.group(2).strip() if match.group(2) else ""
                # Avoid duplicates (same chapter on consecutive pages)
                if chapters and chapters[-1]['chapter_num'] == chapter_num:
                    continue
                chapters.append({
                    'chapter_num': chapter_num,
                    'title': title,
                    'page': i,
                })
        return chapters

    def find_chapter_range(self, chapter_num, chapter_index=None):
        """
        Find the start and end page of a specific chapter.

        Args:
            chapter_num: The chapter number to find.
            chapter_index: Pre-built chapter index. If None, builds one.

        Returns:
            tuple (start_page, end_page) or None if not found.
        """
        if chapter_index is None:
            chapter_index = self.build_chapter_index()

        start_page = None
        end_page = None

        for i, ch in enumerate(chapter_index):
            if ch['chapter_num'] == chapter_num:
                start_page = ch['page']
                # End page is the start of next chapter (or end of PDF)
                if i + 1 < len(chapter_index):
                    end_page = chapter_index[i + 1]['page']
                else:
                    end_page = self.total_pages
                break

        if start_page is None:
            return None
        return (start_page, end_page)

    def extract_pages(self, start_page, end_page, include_page_markers=True):
        """
        Extract text from a range of pages.

        Args:
            start_page: First page (0-indexed)
            end_page: Last page (exclusive, 0-indexed)
            include_page_markers: Whether to include page number markers

        Returns:
            Extracted text as string
        """
        text = ""
        for i in range(start_page, min(end_page, self.total_pages)):
            page_text = self.reader.pages[i].extract_text()
            if include_page_markers:
                text += f"\n{'='*60}\n[PDF Page {i} | Book Page ~{i-10}]\n{'='*60}\n"
            text += page_text + "\n"
        return text

    def extract_chapter(self, chapter_num):
        """
        Extract the full text of a specific chapter.

        Args:
            chapter_num: Chapter number to extract

        Returns:
            Chapter text or error message
        """
        chapter_range = self.find_chapter_range(chapter_num)
        if chapter_range is None:
            # Fallback: try searching for the chapter heading directly
            return self._fallback_chapter_search(chapter_num)

        start, end = chapter_range
        return self.extract_pages(start, end)

    def _fallback_chapter_search(self, chapter_num):
        """Fallback method to find chapter by searching page content."""
        patterns = [
            f"CHAPTER {chapter_num}",
            f"Chapter {chapter_num}",
            f"chapter {chapter_num}",
        ]

        start_page = None
        for i in range(self.total_pages):
            text = self.reader.pages[i].extract_text()
            if not text:
                continue
            for pattern in patterns:
                if pattern in text[:300]:
                    start_page = i
                    break
            if start_page:
                break

        if start_page is None:
            return f"ERROR: Chapter {chapter_num} not found in PDF."

        # Find end by looking for next chapter
        end_page = self.total_pages
        next_patterns = [
            f"CHAPTER {chapter_num + 1}",
            f"Chapter {chapter_num + 1}",
        ]
        for i in range(start_page + 5, self.total_pages):
            text = self.reader.pages[i].extract_text()
            if not text:
                continue
            for pattern in next_patterns:
                if pattern in text[:300]:
                    end_page = i
                    break
            if end_page != self.total_pages:
                break

        return self.extract_pages(start_page, end_page)

    def search(self, query, case_sensitive=False):
        """
        Search for text across all pages.

        Args:
            query: Text to search for
            case_sensitive: Whether search is case-sensitive

        Returns:
            List of dicts with 'page', 'context' (surrounding text)
        """
        results = []
        search_query = query if case_sensitive else query.lower()

        for i in range(self.total_pages):
            text = self.reader.pages[i].extract_text()
            if not text:
                continue
            search_text = text if case_sensitive else text.lower()

            if search_query in search_text:
                # Extract context around the match
                idx = search_text.find(search_query)
                context_start = max(0, idx - 100)
                context_end = min(len(text), idx + len(query) + 100)
                context = text[context_start:context_end]
                results.append({
                    'page': i,
                    'context': context.replace('\n', ' '),
                })
        return results


def main():
    _fix_encoding()
    parser = argparse.ArgumentParser(
        description='PDF Reader Skill - Extract and analyze PDF content',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('pdf_path', help='Path to the PDF file')
    parser.add_argument('command', choices=['info', 'toc', 'chapter', 'pages', 'search', 'index'],
                       help='Command to execute')
    parser.add_argument('args', nargs='*', help='Additional arguments for the command')

    args = parser.parse_args()

    reader = PDFReader(args.pdf_path)

    if args.command == 'info':
        info = reader.get_info()
        print("PDF Information:")
        print("-" * 40)
        for key, value in info.items():
            print(f"  {key}: {value}")

    elif args.command == 'toc':
        # First try bookmarks
        bookmarks = reader.get_bookmarks()
        if bookmarks:
            print("Table of Contents (from bookmarks):")
            print("=" * 60)
            for bm in bookmarks:
                indent = "  " * bm['level']
                page_str = f"p.{bm['page']}" if bm['page'] is not None else "?"
                print(f"{indent}{bm['title']} [{page_str}]")
        else:
            # Fallback to text-based TOC extraction
            print("No bookmarks found. Extracting TOC from text...")
            print("=" * 60)
            toc_range = None
            if args.args and len(args.args) == 2:
                toc_range = (int(args.args[0]), int(args.args[1]))
            toc_text = reader.extract_toc_from_text(toc_range)
            print(toc_text)

    elif args.command == 'chapter':
        if not args.args:
            print("ERROR: Please specify chapter number. Usage: pdf_reader.py book.pdf chapter 14")
            sys.exit(1)
        chapter_num = int(args.args[0])
        print(f"Extracting Chapter {chapter_num}...")
        print("=" * 60)
        text = reader.extract_chapter(chapter_num)
        print(text)

    elif args.command == 'pages':
        if len(args.args) < 2:
            print("ERROR: Please specify start and end pages. Usage: pdf_reader.py book.pdf pages 300 350")
            sys.exit(1)
        start = int(args.args[0])
        end = int(args.args[1])
        text = reader.extract_pages(start, end)
        print(text)

    elif args.command == 'search':
        if not args.args:
            print("ERROR: Please specify search text. Usage: pdf_reader.py book.pdf search \"collision\"")
            sys.exit(1)
        query = " ".join(args.args)
        print(f"Searching for: '{query}'")
        print("=" * 60)
        results = reader.search(query)
        if results:
            print(f"Found {len(results)} page(s) with matches:\n")
            for r in results:
                print(f"  [Page {r['page']}]: ...{r['context']}...")
                print()
        else:
            print("No results found.")

    elif args.command == 'index':
        print("Building chapter index...")
        print("=" * 60)
        chapters = reader.build_chapter_index()
        if chapters:
            print(f"{'Ch#':<5} {'Page':<8} {'Title'}")
            print("-" * 60)
            for ch in chapters:
                print(f"{ch['chapter_num']:<5} {ch['page']:<8} {ch['title']}")
        else:
            print("No chapters found with default pattern.")
            print("Try adjusting the chapter_pattern in the script.")


if __name__ == '__main__':
    main()

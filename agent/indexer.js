const fs = require('fs').promises;
const path = require('path');
const pdfParse = require('pdf-parse');

class Indexer {
  constructor(embeddings) {
    this.embeddings = embeddings;
    this.supportedExtensions = ['.md', '.mdx', '.txt', '.json', '.pdf'];
  }

  // Check if a file has a supported extension
  isSupportedFile(filePath) {
    const ext = path.extname(filePath).toLowerCase();
    return this.supportedExtensions.includes(ext);
  }

  // Read content from a file based on its extension
  async readFileContent(filePath) {
    const ext = path.extname(filePath).toLowerCase();

    try {
      if (ext === '.pdf') {
        // Handle PDF files
        const dataBuffer = await fs.readFile(filePath);
        const pdfData = await pdfParse(dataBuffer);
        return pdfData.text;
      } else {
        // Handle text-based files
        let content = await fs.readFile(filePath, 'utf8');

        // Remove frontmatter from markdown files
        if (ext === '.md' || ext === '.mdx') {
          content = content.replace(/^---[\s\S]*?---\s*/, '');
        }

        return content;
      }
    } catch (error) {
      console.error(`Error reading file ${filePath}:`, error.message);
      return null;
    }
  }

  // Recursively walk through a directory and index all supported files
  async indexDirectory(dirPath) {
    const files = await fs.readdir(dirPath);
    let indexedCount = 0;

    for (const file of files) {
      const filePath = path.join(dirPath, file);
      const stat = await fs.stat(filePath);

      if (stat.isDirectory()) {
        // Recursively index subdirectories
        indexedCount += await this.indexDirectory(filePath);
      } else if (this.isSupportedFile(filePath)) {
        // Index supported files
        const content = await this.readFileContent(filePath);
        if (content) {
          const relativePath = path.relative(process.cwd(), filePath);
          this.embeddings.addDocument(relativePath, content, {
            path: relativePath,
            title: path.basename(filePath)
          });
          console.log(`Indexed: ${relativePath}`);
          indexedCount++;
        }
      }
    }

    return indexedCount;
  }

  // Index content from both book and docs directories
  async indexContent() {
    console.log('Starting documentation indexing...');

    let totalIndexed = 0;

    // Index book directory if it exists
    const bookDir = path.join(process.cwd(), 'book');
    if (await this.pathExists(bookDir)) {
      totalIndexed += await this.indexDirectory(bookDir);
    }

    // Index docs directory
    const docsDir = path.join(process.cwd(), 'docs');
    if (await this.pathExists(docsDir)) {
      totalIndexed += await this.indexDirectory(docsDir);
    }

    console.log(`Indexed: ${totalIndexed} documents`);
    return totalIndexed;
  }

  // Helper function to check if a path exists
  async pathExists(filePath) {
    try {
      await fs.access(filePath);
      return true;
    } catch {
      return false;
    }
  }
}

module.exports = Indexer;
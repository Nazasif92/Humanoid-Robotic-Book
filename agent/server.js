const express = require('express');

class Server {
  constructor(embeddings, indexer) {
    this.embeddings = embeddings;
    this.indexer = indexer;
    this.app = express();
    this.setupMiddleware();
    this.setupRoutes();
  }

  // Setup middleware
  setupMiddleware() {
    this.app.use(express.json({ limit: '10mb' }));
    this.app.use(express.static('public'));
  }

  // Setup API routes
  setupRoutes() {
    // POST /ask - Answer question from indexed content
    this.app.post('/ask', async (req, res) => {
      try {
        const { question } = req.body;

        if (!question) {
          return res.status(400).json({ error: 'Question is required' });
        }

        // Search for relevant documents
        const results = this.embeddings.search(question, 5);

        if (results.length === 0) {
          return res.json({
            question,
            answer: "I couldn't find any relevant information in the indexed content to answer your question.",
            sources: []
          });
        }

        // Generate answer from most relevant results
        const topResults = results.slice(0, 3);
        const answer = this.generateAnswer(question, topResults);

        res.json({
          question,
          answer,
          sources: topResults.map(r => ({
            path: r.metadata.path,
            title: r.metadata.title,
            similarity: r.similarity
          }))
        });
      } catch (error) {
        console.error('Error processing question:', error);
        res.status(500).json({ error: 'Internal server error' });
      }
    });

    // GET /status - Return number of indexed documents
    this.app.get('/status', (req, res) => {
      res.json({
        indexedDocuments: this.embeddings.getDocumentCount(),
        status: 'ready'
      });
    });

    // GET /search?q= - Search for matching text chunks
    this.app.get('/search', (req, res) => {
      const { q } = req.query;

      if (!q) {
        return res.status(400).json({ error: 'Query parameter "q" is required' });
      }

      const results = this.embeddings.search(q, 10);

      res.json({
        query: q,
        results: results.map(r => ({
          content: r.content.substring(0, 500) + (r.content.length > 500 ? '...' : ''),
          path: r.metadata.path,
          title: r.metadata.title,
          similarity: r.similarity
        }))
      });
    });

    // Serve the main page
    this.app.get('/', (req, res) => {
      res.sendFile('index.html');
    });
  }

  // Generate answer from search results
  generateAnswer(question, results) {
    if (results.length === 0) {
      return "I couldn't find any relevant information to answer your question.";
    }

    // Combine the most relevant content snippets
    const context = results
      .map(r => `From ${r.metadata.title}: ${r.content.substring(0, 300)}`)
      .join('\n\n');

    // Simple answer generation based on context
    return `Based on the documentation:\n\n${context}\n\nThis information was found in ${results.length} document(s).`;
  }

  // Start the server
  start(port = 3002) {
    this.app.listen(port, () => {
      console.log(`Agent server running on port ${port}`);
      console.log(`Chat UI ready at http://localhost:${port}`);
    });
  }
}

module.exports = Server;
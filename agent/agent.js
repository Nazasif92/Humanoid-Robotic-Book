const Embeddings = require('./embeddings');
const Indexer = require('./indexer');
const Server = require('./server');

async function main() {
  // Create embeddings instance
  const embeddings = new Embeddings();

  // Create indexer instance
  const indexer = new Indexer(embeddings);

  // Index the content
  await indexer.indexContent();

  // Create and start server
  const server = new Server(embeddings, indexer);
  server.start(3002);
}

// Handle uncaught exceptions
process.on('uncaughtException', (err) => {
  console.error('Uncaught Exception:', err);
});

process.on('unhandledRejection', (reason, promise) => {
  console.error('Unhandled Rejection at:', promise, 'reason:', reason);
});

// Run the main function
main().catch(console.error);
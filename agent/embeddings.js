// Simple text embedding using TF-IDF approach for similarity matching
class Embeddings {
  constructor() {
    this.documents = [];
  }

  // Tokenize text into words
  tokenize(text) {
    return text.toLowerCase()
      .replace(/[^\w\s]/g, ' ')
      .split(/\s+/)
      .filter(token => token.length > 0);
  }

  // Calculate term frequency for a document
  calculateTF(tokens) {
    const tf = {};
    const totalTokens = tokens.length;

    for (const token of tokens) {
      tf[token] = (tf[token] || 0) + 1;
    }

    // Normalize by document length
    for (const token in tf) {
      tf[token] /= totalTokens;
    }

    return tf;
  }

  // Calculate cosine similarity between two vectors
  cosineSimilarity(vecA, vecB) {
    const tokensA = new Set(Object.keys(vecA));
    const tokensB = new Set(Object.keys(vecB));

    const intersection = [...tokensA].filter(token => tokensB.has(token));

    if (intersection.length === 0) return 0;

    let dotProduct = 0;
    let normA = 0;
    let normB = 0;

    for (const token of tokensA) {
      const aVal = vecA[token] || 0;
      const bVal = vecB[token] || 0;
      dotProduct += aVal * bVal;
      normA += aVal * aVal;
    }

    for (const token of tokensB) {
      const bVal = vecB[token] || 0;
      normB += bVal * bVal;
    }

    if (normA === 0 || normB === 0) return 0;

    return dotProduct / (Math.sqrt(normA) * Math.sqrt(normB));
  }

  // Add a document to the index
  addDocument(id, content, metadata = {}) {
    const tokens = this.tokenize(content);
    const tf = this.calculateTF(tokens);

    this.documents.push({
      id,
      content,
      tokens,
      tf,
      metadata
    });
  }

  // Search for similar documents to the query
  search(query, topK = 5) {
    const queryTokens = this.tokenize(query);
    const queryTF = this.calculateTF(queryTokens);

    const similarities = this.documents.map(doc => {
      const similarity = this.cosineSimilarity(queryTF, doc.tf);
      return {
        id: doc.id,
        content: doc.content,
        similarity,
        metadata: doc.metadata
      };
    });

    // Sort by similarity and return top K
    return similarities
      .sort((a, b) => b.similarity - a.similarity)
      .slice(0, topK)
      .filter(result => result.similarity > 0); // Only return results with some similarity
  }

  // Clear the document index
  clear() {
    this.documents = [];
  }

  // Get the number of indexed documents
  getDocumentCount() {
    return this.documents.length;
  }
}

module.exports = Embeddings;
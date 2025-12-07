import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';
import styles from './chatbot.module.css';

/**
 * Chatbot Page Component
 *
 * Provides a conversational UI for asking questions about the documentation.
 * Features:
 * - Question input with submit button
 * - Streaming/delayed answer display
 * - Source citations with links
 * - Optional selected text context
 * - Error handling and loading states
 */

const BACKEND_URL = process.env.NODE_ENV === 'production'
  ? 'https://rag-chatbot-backend.railway.app'
  : 'http://localhost:8000';

export default function ChatbotPage() {
  const [question, setQuestion] = useState('');
  const [selectedText, setSelectedText] = useState('');
  const [answer, setAnswer] = useState('');
  const [sources, setSources] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [latency, setLatency] = useState(null);
  const [history, setHistory] = useState([]);
  const answerRef = useRef(null);
  const messagesEndRef = useRef(null);

  // Load history from localStorage on mount
  useEffect(() => {
    const saved = localStorage.getItem('chatbot_history');
    if (saved) {
      try {
        setHistory(JSON.parse(saved));
      } catch (e) {
        console.error('Failed to load history:', e);
      }
    }
  }, []);

  // Auto-scroll to latest message
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  }, [answer, history]);

  /**
   * Submit question to backend
   */
  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!question.trim()) {
      setError('Please enter a question');
      return;
    }

    setError('');
    setLoading(true);
    setAnswer('');
    setSources([]);
    setLatency(null);

    try {
      const response = await fetch(`${BACKEND_URL}/ask`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          question: question.trim(),
          selected_text: selectedText || undefined,
        }),
      });

      if (!response.ok) {
        throw new Error(`Server error: ${response.status}`);
      }

      const data = await response.json();

      if (data.status === 'success') {
        setAnswer(data.answer);
        setSources(data.sources || []);
        setLatency(data.latency_ms);

        // Save to history
        const newEntry = {
          id: Date.now(),
          question: question.trim(),
          answer: data.answer,
          sources: data.sources,
          timestamp: new Date().toLocaleString(),
        };
        const newHistory = [newEntry, ...history].slice(0, 50); // Keep last 50
        setHistory(newHistory);
        localStorage.setItem('chatbot_history', JSON.stringify(newHistory));

        // Clear inputs
        setQuestion('');
        setSelectedText('');
      } else {
        setError(data.error || 'Failed to generate answer');
      }
    } catch (err) {
      setError(`Error: ${err.message || 'Failed to connect to backend'}`);
      console.error('Request failed:', err);
    } finally {
      setLoading(false);
    }
  };

  /**
   * Load a history entry
   */
  const handleLoadHistory = (entry) => {
    setAnswer(entry.answer);
    setSources(entry.sources);
    setQuestion(entry.question);
    setError('');
    setLatency(null);
  };

  /**
   * Detect selected text on the page
   */
  const handleUseSelection = () => {
    const selected = window.getSelection().toString();
    if (selected) {
      setSelectedText(selected.substring(0, 2000)); // Limit to 2000 chars
    } else {
      setError('Please select text on the page first');
    }
  };

  return (
    <Layout
      title="Chatbot"
      description="Ask questions about the Humanoid Robotics Book documentation"
    >
      <div className={styles.container}>
        <div className={styles.chatContainer}>
          {/* Header */}
          <div className={styles.header}>
            <h1>📚 Documentation Chatbot</h1>
            <p>Ask questions about the Humanoid Robotics Book</p>
          </div>

          {/* Main Chat Area */}
          <div className={styles.chatArea}>
            {/* Message History */}
            {history.length > 0 && (
              <div className={styles.historySection}>
                <h3>Recent Conversations</h3>
                <div className={styles.historyList}>
                  {history.slice(0, 5).map((entry) => (
                    <button
                      key={entry.id}
                      className={styles.historyItem}
                      onClick={() => handleLoadHistory(entry)}
                      title={entry.question}
                    >
                      <span className={styles.historyQuestion}>
                        {entry.question.substring(0, 50)}...
                      </span>
                      <span className={styles.historyTime}>
                        {entry.timestamp}
                      </span>
                    </button>
                  ))}
                </div>
              </div>
            )}

            {/* Answer Display */}
            {answer && (
              <div className={styles.answerSection} ref={answerRef}>
                <div className={styles.answerBox}>
                  <h3>Answer:</h3>
                  <p className={styles.answerText}>{answer}</p>
                  {latency && (
                    <p className={styles.latency}>
                      Generated in {latency}ms
                    </p>
                  )}
                </div>

                {/* Sources */}
                {sources.length > 0 && (
                  <div className={styles.sourcesSection}>
                    <h4>Sources:</h4>
                    <ul className={styles.sourcesList}>
                      {sources.map((source, idx) => (
                        <li key={idx} className={styles.sourceItem}>
                          <strong>{source.title}</strong>
                          {' '}
                          <span className={styles.section}>
                            ({source.section})
                          </span>
                          <br />
                          <a
                            href={source.url}
                            target="_blank"
                            rel="noopener noreferrer"
                            className={styles.sourceLink}
                          >
                            Read more →
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
              </div>
            )}

            {/* Loading State */}
            {loading && (
              <div className={styles.loadingSection}>
                <div className={styles.spinner} />
                <p>Searching documentation...</p>
              </div>
            )}

            {/* Error State */}
            {error && (
              <div className={styles.errorBox}>
                <p>⚠️ {error}</p>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          {/* Input Section */}
          <form onSubmit={handleSubmit} className={styles.inputSection}>
            <div className={styles.inputGroup}>
              {/* Selected Text Input */}
              {selectedText && (
                <div className={styles.selectedTextBox}>
                  <label>Context (selected text):</label>
                  <textarea
                    value={selectedText}
                    onChange={(e) => setSelectedText(e.target.value)}
                    placeholder="Selected text context..."
                    className={styles.contextInput}
                  />
                  <button
                    type="button"
                    onClick={() => setSelectedText('')}
                    className={styles.clearButton}
                  >
                    Clear context
                  </button>
                </div>
              )}

              {/* Question Input */}
              <div className={styles.questionGroup}>
                <textarea
                  value={question}
                  onChange={(e) => setQuestion(e.target.value)}
                  placeholder="Ask a question about the documentation..."
                  className={styles.questionInput}
                  disabled={loading}
                  rows={3}
                />
                <div className={styles.buttonGroup}>
                  <button
                    type="submit"
                    disabled={loading}
                    className={styles.submitButton}
                  >
                    {loading ? 'Loading...' : '🚀 Ask'}
                  </button>
                  <button
                    type="button"
                    onClick={handleUseSelection}
                    disabled={loading}
                    className={styles.contextButton}
                    title="Select text on the page first, then click this button"
                  >
                    📝 Use Selection
                  </button>
                </div>
              </div>
            </div>
          </form>

          {/* Footer */}
          <div className={styles.footer}>
            <p>
              💡 <strong>Tip:</strong> Select text on any documentation page and click "Use Selection" to ask context-specific questions
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}

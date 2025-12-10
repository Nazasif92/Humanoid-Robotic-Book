import React, { useState, useRef, useEffect } from 'react';
import { ragClient } from '../../api/rag-client';
import styles from './ChatBox.module.css';

/**
 * ChatBox Component
 *
 * A React component that provides an interface for users to ask questions
 * to the RAG backend and receive AI-generated answers with source citations.
 *
 * @param {Object} props - Component properties
 * @param {string} [props.initialContext] - Initial context to pre-populate the component
 * @param {boolean} [props.showTitle=true] - Whether to show the component title
 * @param {boolean} [props.compact=false] - Whether to use compact styling
 * @param {Function} [props.onAnswer] - Callback function when an answer is received
 */
const ChatBox = ({
  initialContext = '',
  showTitle = true,
  compact = false,
  onAnswer = null
}) => {
  const [question, setQuestion] = useState('');
  const [context, setContext] = useState(initialContext);
  const [answer, setAnswer] = useState('');
  const [sources, setSources] = useState([]);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState('');
  const [latency, setLatency] = useState(null);
  const [showContextInput, setShowContextInput] = useState(!!initialContext);

  const answerRef = useRef(null);
  const questionInputRef = useRef(null);

  // Auto-scroll to answer when it updates
  useEffect(() => {
    if (answerRef.current && answer) {
      answerRef.current.scrollIntoView({ behavior: 'smooth', block: 'start' });
    }
  }, [answer]);

  // Handle form submission
  const handleSubmit = async (e) => {
    e.preventDefault();

    if (!question.trim()) {
      setError('Please enter a question');
      questionInputRef.current?.focus();
      return;
    }

    setError('');
    setLoading(true);
    setAnswer('');
    setSources([]);
    setLatency(null);

    try {
      const startTime = Date.now();
      const response = await ragClient.ask(question, context);
      const endTime = Date.now();

      setLatency(endTime - startTime);

      if (response.status === 'success') {
        setAnswer(response.answer);
        setSources(response.sources || []);

        // Call the onAnswer callback if provided
        if (onAnswer) {
          onAnswer({
            question,
            answer: response.answer,
            sources: response.sources || [],
            latency: endTime - startTime
          });
        }
      } else {
        setError(response.error || 'Failed to generate answer');
      }
    } catch (err) {
      setError(err.message || 'An error occurred while processing your question');
      console.error('ChatBox error:', err);
    } finally {
      setLoading(false);
      // Focus back to question input after submission
      if (!answer) {
        questionInputRef.current?.focus();
      }
    }
  };

  // Handle text selection context
  const handleUseSelection = () => {
    if (typeof window !== 'undefined') {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        setContext(selectedText.substring(0, 2000)); // Limit to 2000 chars
        setShowContextInput(true);
        // Focus on the question input after setting context
        setTimeout(() => questionInputRef.current?.focus(), 100);
      } else {
        setError('Please select text on the page first');
        // Focus on the question input after error
        setTimeout(() => questionInputRef.current?.focus(), 100);
      }
    }
  };

  return (
    <div
      className={`${styles.chatBox} ${compact ? styles.compact : ''}`}
      role="region"
      aria-label="Documentation Q&A interface"
    >
      {showTitle && (
        <h3 className={styles.title} id="chatbox-title">
          🤖 Ask about the Documentation
        </h3>
      )}

      <form onSubmit={handleSubmit} className={styles.form} aria-labelledby="chatbox-title">
        {/* Context Input (for selected text or provided context) */}
        {showContextInput && (
          <div className={styles.contextSection} role="group" aria-labelledby="context-label">
            <label id="context-label" className={styles.contextLabel}>
              Context:
            </label>
            <textarea
              value={context}
              onChange={(e) => setContext(e.target.value.substring(0, 2000))}
              placeholder="Selected text context..."
              className={styles.contextInput}
              rows={Math.min(4, Math.max(2, context.split('\n').length))}
              aria-describedby="context-help"
              aria-label="Context for your question"
            />
            <div id="context-help" className="sr-only">
              Provide context for your question, limited to 2000 characters
            </div>
            <button
              type="button"
              onClick={() => {
                setContext('');
                setShowContextInput(false);
                questionInputRef.current?.focus();
              }}
              className={styles.clearContextButton}
              aria-label="Clear context"
            >
              Clear Context
            </button>
          </div>
        )}

        {/* Question Input */}
        <div className={styles.inputGroup}>
          <textarea
            ref={questionInputRef}
            value={question}
            onChange={(e) => setQuestion(e.target.value)}
            placeholder="Ask a question about the documentation..."
            className={styles.questionInput}
            rows={compact ? 2 : 3}
            disabled={loading}
            aria-label="Your question"
            aria-describedby={error ? "chatbox-error" : undefined}
            autoFocus={!initialContext} // Auto-focus on initial render if no initial context
          />
          <div className={styles.buttonGroup} role="group" aria-label="Chat actions">
            <button
              type="button"
              onClick={handleUseSelection}
              className={styles.contextButton}
              disabled={loading}
              title="Use selected text as context"
              aria-label="Use selected text as context"
            >
              📝
            </button>
            <button
              type="submit"
              disabled={loading}
              className={styles.submitButton}
              aria-label={loading ? "Sending question..." : "Submit question"}
            >
              {loading ? 'Sending...' : '🚀 Ask'}
            </button>
          </div>
        </div>
      </form>

      {/* Error Display */}
      {error && (
        <div
          className={styles.errorBox}
          role="alert"
          id="chatbox-error"
          aria-live="polite"
        >
          <p>{error}</p>
        </div>
      )}

      {/* Loading State */}
      {loading && (
        <div
          className={styles.loadingBox}
          role="status"
          aria-live="polite"
        >
          <div className={styles.spinner} aria-hidden="true"></div>
          <p>Thinking...</p>
        </div>
      )}

      {/* Answer Display */}
      {(answer || sources.length > 0) && (
        <div
          ref={answerRef}
          className={styles.responseSection}
          role="region"
          aria-labelledby="answer-heading"
          tabIndex="-1" // Make it focusable for screen readers when content updates
        >
          {/* Answer */}
          {answer && (
            <div className={styles.answerBox}>
              <h4 id="answer-heading">Answer:</h4>
              <div
                className={styles.answerText}
                dangerouslySetInnerHTML={{ __html: answer.replace(/\n/g, '<br />') }}
              />
              {latency && (
                <div className={styles.latencyInfo} aria-label={`Generated in ${latency} milliseconds`}>
                  Generated in {latency}ms
                </div>
              )}
            </div>
          )}

          {/* Sources */}
          {sources.length > 0 && (
            <div className={styles.sourcesBox}>
              <h4>Sources:</h4>
              <ul className={styles.sourcesList} aria-label="Sources for this answer">
                {sources.map((source, index) => (
                  <li key={index} className={styles.sourceItem}>
                    <div className={styles.sourceTitle}>
                      <strong>{source.title}</strong>
                      {source.section && <span className={styles.sourceSection}> ({source.section})</span>}
                    </div>
                    {source.chunk_text && (
                      <div className={styles.sourceExcerpt} aria-label={`Source excerpt: ${source.chunk_text.substring(0, 150)}${source.chunk_text.length > 150 ? '...' : ''}`}>
                        "{source.chunk_text.substring(0, 150)}{source.chunk_text.length > 150 ? '...' : ''}"
                      </div>
                    )}
                    <a
                      href={source.url}
                      target="_blank"
                      rel="noopener noreferrer"
                      className={styles.sourceLink}
                      aria-label={`Read more about ${source.title} in a new tab`}
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
    </div>
  );
};

export default ChatBox;
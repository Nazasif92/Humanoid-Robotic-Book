import React, { useState, useEffect, useCallback } from 'react';
import ChatBox from '../ChatBox/ChatBox';
import styles from './TextSelectionChat.module.css';

/**
 * TextSelectionChat Component
 *
 * A component that detects text selection on the page and provides
 * an interface to ask questions about the selected text using the RAG backend.
 * It creates a floating button that appears when text is selected.
 */
const TextSelectionChat = () => {
  const [selectedText, setSelectedText] = useState('');
  const [showChat, setShowChat] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });

  // Handle text selection
  const handleSelection = useCallback(() => {
    if (typeof window !== 'undefined') {
      const selection = window.getSelection();
      const text = selection.toString().trim();

      if (text && text.length > 0) {
        // Limit selected text to 2000 characters (same as backend limit)
        const limitedText = text.substring(0, 2000);

        // Get cursor position to show the button nearby
        const range = selection.getRangeAt(0);
        const rect = range.getBoundingClientRect();

        setSelectedText(limitedText);
        setPosition({ x: rect.left, y: rect.top - 10 });
        setShowChat(true);
      } else {
        setShowChat(false);
      }
    }
  }, []);

  // Add event listeners
  useEffect(() => {
    // Add mouseup event to detect when text selection is complete
    document.addEventListener('mouseup', handleSelection);

    // Also listen for selectionchange for more comprehensive detection
    document.addEventListener('selectionchange', handleSelection);

    // Cleanup event listeners
    return () => {
      document.removeEventListener('mouseup', handleSelection);
      document.removeEventListener('selectionchange', handleSelection);
    };
  }, [handleSelection]);

  // Handle closing the chat
  const handleClose = () => {
    setShowChat(false);
    setSelectedText('');
  };

  // Prevent interaction with the chat when clicking on it
  const handleChatClick = (e) => {
    e.stopPropagation();
  };

  if (!showChat) {
    return null;
  }

  return (
    <div
      className={styles.textSelectionChat}
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
        position: 'fixed',
        zIndex: 10000
      }}
      onClick={handleChatClick}
    >
      <div className={styles.chatContainer}>
        <div className={styles.header}>
          <h4>Ask about selected text</h4>
          <button
            className={styles.closeButton}
            onClick={handleClose}
            aria-label="Close"
          >
            ×
          </button>
        </div>
        <ChatBox
          initialContext={selectedText}
          showTitle={false}
          onAnswer={() => {
            // Optionally clear the selection after asking
            if (typeof window !== 'undefined') {
              window.getSelection().removeAllRanges();
            }
          }}
        />
      </div>
    </div>
  );
};

export default TextSelectionChat;
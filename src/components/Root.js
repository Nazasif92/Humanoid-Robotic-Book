import React from 'react';
import TextSelectionChat from './TextSelectionChat/TextSelectionChat';

/**
 * Root Component
 *
 * A Docusaurus Root component that wraps the entire application
 * and provides global functionality like text selection chat.
 */
const Root = ({ children }) => {
  return (
    <>
      {children}
      <TextSelectionChat />
    </>
  );
};

export default Root;
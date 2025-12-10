import React from 'react';
import TextSelectionChat from '../components/TextSelectionChat/TextSelectionChat';

/**
 * AskOnSelectedText Component
 *
 * A Docusaurus theme component that enables the text selection chat feature
 * on documentation pages. This component renders the TextSelectionChat component
 * which detects text selection and provides an interface to ask questions.
 */
const AskOnSelectedText = () => {
  return <TextSelectionChat />;
};

export default AskOnSelectedText;
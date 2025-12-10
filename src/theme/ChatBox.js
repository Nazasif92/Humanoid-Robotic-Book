import React from 'react';
import ChatBoxComponent from '../components/ChatBox/ChatBox';

/**
 * ChatBox Theme Component
 *
 * A Docusaurus theme component that wraps the ChatBox component
 * for easy use in MDX files and documentation pages.
 */
const ChatBox = (props) => {
  return <ChatBoxComponent {...props} />;
};

export default ChatBox;
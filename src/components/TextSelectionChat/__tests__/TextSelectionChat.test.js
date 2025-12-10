import React from 'react';
import { render, screen } from '@testing-library/react';
import TextSelectionChat from '../TextSelectionChat';

// Mock the ChatBox component
jest.mock('../../components/ChatBox/ChatBox', () => {
  return function DummyChatBox(props) {
    return <div data-testid="chatbox-mock" {...props}>ChatBox Mock</div>;
  };
});

describe('TextSelectionChat Component', () => {
  beforeEach(() => {
    // Clear any previous mocks
    jest.clearAllMocks();

    // Mock window.getSelection
    Object.defineProperty(window, 'getSelection', {
      writable: true,
      value: jest.fn(() => ({
        toString: () => '',
        removeAllRanges: jest.fn(),
        getRangeAt: jest.fn(() => ({
          getBoundingClientRect: () => ({ left: 0, top: 0 })
        }))
      }))
    });
  });

  it('renders without crashing', () => {
    render(<TextSelectionChat />);
    // Initially, the component should not render anything if no text is selected
    expect(screen.queryByTestId('chatbox-mock')).not.toBeInTheDocument();
  });

  it('does not show chat when no text is selected', () => {
    // Mock selection to return empty string
    const mockSelection = {
      toString: () => '',
      removeAllRanges: jest.fn(),
      getRangeAt: jest.fn(() => ({
        getBoundingClientRect: () => ({ left: 0, top: 0 })
      }))
    };

    Object.defineProperty(window, 'getSelection', {
      writable: true,
      value: jest.fn(() => mockSelection)
    });

    render(<TextSelectionChat />);

    // The component should not render the chatbox when no text is selected
    expect(screen.queryByTestId('chatbox-mock')).not.toBeInTheDocument();
  });

  it('has proper event listeners setup', () => {
    // This test verifies that the component sets up the required event listeners
    const addEventListenerSpy = jest.spyOn(document, 'addEventListener');
    const removeEventListenerSpy = jest.spyOn(document, 'removeEventListener');

    const { unmount } = render(<TextSelectionChat />);

    // Check that event listeners were added
    expect(addEventListenerSpy).toHaveBeenCalledWith('mouseup', expect.any(Function));
    expect(addEventListenerSpy).toHaveBeenCalledWith('selectionchange', expect.any(Function));

    // Unmount to trigger cleanup
    unmount();

    // Check that event listeners were removed
    expect(removeEventListenerSpy).toHaveBeenCalledWith('mouseup', expect.any(Function));
    expect(removeEventListenerSpy).toHaveBeenCalledWith('selectionchange', expect.any(Function));

    addEventListenerSpy.mockRestore();
    removeEventListenerSpy.mockRestore();
  });
});
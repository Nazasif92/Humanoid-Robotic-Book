import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import ChatBox from '../ChatBox';

// Mock the RAG client
jest.mock('../../../api/rag-client', () => ({
  ragClient: {
    ask: jest.fn(),
    health: jest.fn()
  }
}));

import { ragClient } from '../../../api/rag-client';

describe('ChatBox Component', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  it('renders without crashing', () => {
    render(<ChatBox />);
    expect(screen.getByPlaceholderText('Ask a question about the documentation...')).toBeInTheDocument();
  });

  it('shows the title by default', () => {
    render(<ChatBox />);
    expect(screen.getByText('🤖 Ask about the Documentation')).toBeInTheDocument();
  });

  it('hides the title when showTitle is false', () => {
    render(<ChatBox showTitle={false} />);
    expect(screen.queryByText('🤖 Ask about the Documentation')).not.toBeInTheDocument();
  });

  it('submits a question to the RAG backend', async () => {
    const mockResponse = {
      answer: 'This is a test answer',
      sources: [],
      status: 'success'
    };

    ragClient.ask.mockResolvedValue(mockResponse);

    render(<ChatBox />);

    const questionInput = screen.getByPlaceholderText('Ask a question about the documentation...');
    const submitButton = screen.getByText('🚀 Ask');

    fireEvent.change(questionInput, { target: { value: 'Test question?' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(ragClient.ask).toHaveBeenCalledWith('Test question?', null);
      expect(screen.getByText('Answer:')).toBeInTheDocument();
      expect(screen.getByText('This is a test answer')).toBeInTheDocument();
    });
  });

  it('includes context when provided', async () => {
    const mockResponse = {
      answer: 'This is an answer with context',
      sources: [],
      status: 'success'
    };

    ragClient.ask.mockResolvedValue(mockResponse);

    render(<ChatBox initialContext="This is context" />);

    const questionInput = screen.getByPlaceholderText('Ask a question about the documentation...');
    const submitButton = screen.getByText('🚀 Ask');

    fireEvent.change(questionInput, { target: { value: 'Test question?' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(ragClient.ask).toHaveBeenCalledWith('Test question?', 'This is context');
    });
  });

  it('shows error message for empty questions', async () => {
    render(<ChatBox />);

    const submitButton = screen.getByText('🚀 Ask');
    fireEvent.click(submitButton);

    expect(screen.getByText('Please enter a question')).toBeInTheDocument();
  });

  it('displays sources when returned by the API', async () => {
    const mockResponse = {
      answer: 'This is a test answer',
      sources: [
        {
          title: 'Test Document',
          section: 'Getting Started',
          url: 'https://example.com/test',
          chunk_text: 'This is a sample chunk of text from the document.'
        }
      ],
      status: 'success'
    };

    ragClient.ask.mockResolvedValue(mockResponse);

    render(<ChatBox />);

    const questionInput = screen.getByPlaceholderText('Ask a question about the documentation...');
    const submitButton = screen.getByText('🚀 Ask');

    fireEvent.change(questionInput, { target: { value: 'Test question?' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(screen.getByText('Sources:')).toBeInTheDocument();
      expect(screen.getByText('Test Document')).toBeInTheDocument();
      expect(screen.getByText('(Getting Started)')).toBeInTheDocument();
      expect(screen.getByText('Read more →')).toBeInTheDocument();
    });
  });

  it('shows loading state during API request', async () => {
    // Make the API call take some time to complete
    ragClient.ask.mockImplementation(() => new Promise(resolve => {
      setTimeout(() => resolve({
        answer: 'Delayed answer',
        sources: [],
        status: 'success'
      }), 100);
    }));

    render(<ChatBox />);

    const questionInput = screen.getByPlaceholderText('Ask a question about the documentation...');
    const submitButton = screen.getByText('🚀 Ask');

    fireEvent.change(questionInput, { target: { value: 'Test question?' } });
    fireEvent.click(submitButton);

    // Check that loading state is shown immediately
    expect(screen.getByText('Thinking...')).toBeInTheDocument();

    // Wait for the response to complete
    await waitFor(() => {
      expect(screen.getByText('Delayed answer')).toBeInTheDocument();
    });
  });

  it('calls onAnswer callback when provided', async () => {
    const onAnswerMock = jest.fn();
    const mockResponse = {
      answer: 'Callback test answer',
      sources: [],
      status: 'success'
    };

    ragClient.ask.mockResolvedValue(mockResponse);

    render(<ChatBox onAnswer={onAnswerMock} />);

    const questionInput = screen.getByPlaceholderText('Ask a question about the documentation...');
    const submitButton = screen.getByText('🚀 Ask');

    fireEvent.change(questionInput, { target: { value: 'Test question?' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(onAnswerMock).toHaveBeenCalledWith(
        expect.objectContaining({
          question: 'Test question?',
          answer: 'Callback test answer',
          sources: []
        })
      );
    });
  });

  it('handles API errors gracefully', async () => {
    ragClient.ask.mockRejectedValue(new Error('API Error'));

    render(<ChatBox />);

    const questionInput = screen.getByPlaceholderText('Ask a question about the documentation...');
    const submitButton = screen.getByText('🚀 Ask');

    fireEvent.change(questionInput, { target: { value: 'Test question?' } });
    fireEvent.click(submitButton);

    await waitFor(() => {
      expect(screen.getByText('API Error')).toBeInTheDocument();
    });
  });
});
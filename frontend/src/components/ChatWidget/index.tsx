import React, { useState, useRef, useEffect, useCallback } from 'react';
import styles from './styles.module.css';

interface Source {
  chapter: string;
  section: string;
  content: string;
  url: string;
  relevance_score: number;
}

interface ImageResult {
  id: string;
  url: string;
  title: string;
  alt_text: string;
  chapter: string;
  section: string;
  score: number;
}

interface Message {
  role: 'user' | 'assistant';
  content: string;
  sources?: Source[];
  images?: ImageResult[];
  selectedText?: string;
}

interface SelectionPopup {
  visible: boolean;
  x: number;
  y: number;
  text: string;
}

const API_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://mqasim077-physical-ai-textbook-api.hf.space'; // Hugging Face Spaces URL

export default function ChatWidget(): JSX.Element {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<Message[]>([]);
  const [input, setInput] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [selectionPopup, setSelectionPopup] = useState<SelectionPopup>({
    visible: false,
    x: 0,
    y: 0,
    text: '',
  });
  const messagesEndRef = useRef<HTMLDivElement>(null);
  const inputRef = useRef<HTMLInputElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle text selection
  const handleTextSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();

    if (text && text.length > 0 && text.length < 500) {
      const range = selection?.getRangeAt(0);
      const rect = range?.getBoundingClientRect();

      if (rect) {
        setSelectionPopup({
          visible: true,
          x: rect.left + rect.width / 2,
          y: rect.top - 10,
          text: text,
        });
      }
    } else {
      setSelectionPopup(prev => ({ ...prev, visible: false }));
    }
  }, []);

  // Listen for text selection
  useEffect(() => {
    const handleMouseUp = () => {
      setTimeout(handleTextSelection, 10);
    };

    const handleMouseDown = (e: MouseEvent) => {
      const target = e.target as HTMLElement;
      if (!target.closest(`.${styles.selectionPopup}`)) {
        setSelectionPopup(prev => ({ ...prev, visible: false }));
      }
    };

    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleMouseDown);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleMouseDown);
    };
  }, [handleTextSelection]);

  // Ask about selected text
  const askAboutSelection = () => {
    const text = selectionPopup.text;
    setSelectedText(text);
    setSelectionPopup(prev => ({ ...prev, visible: false }));
    setIsOpen(true);

    // Clear any existing selection
    window.getSelection()?.removeAllRanges();

    // Focus input and set a prompt
    setTimeout(() => {
      inputRef.current?.focus();
      setInput(`Explain this: "${text.substring(0, 100)}${text.length > 100 ? '...' : ''}"`);
    }, 100);
  };

  const sendMessage = async () => {
    if (!input.trim() || isLoading) return;

    const userMessage = input.trim();
    setInput('');
    setMessages(prev => [...prev, { role: 'user', content: userMessage }]);
    setIsLoading(true);

    try {
      const response = await fetch(`${API_URL}/api/chat/`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          message: userMessage,
          history: messages.slice(-4).map(m => ({
            role: m.role,
            content: m.content,
          })),
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to get response');
      }

      const data = await response.json();
      setMessages(prev => [
        ...prev,
        {
          role: 'assistant',
          content: data.answer,
          sources: data.sources,
          images: data.images,
        },
      ]);
    } catch (error) {
      setMessages(prev => [
        ...prev,
        {
          role: 'assistant',
          content: 'Sorry, I encountered an error. Please try again.',
        },
      ]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  // Reset conversation
  const handleNewChat = () => {
    setMessages([]);
    setSelectedText('');
    setInput('');
  };

  return (
    <>
      {/* Selection Popup - Ask AI Button */}
      {selectionPopup.visible && (
        <button
          className={styles.selectionPopup}
          style={{
            left: `${selectionPopup.x}px`,
            top: `${selectionPopup.y}px`,
          }}
          onClick={askAboutSelection}
          aria-label="Ask AI about selected text"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
          Ask AI
        </button>
      )}

      {/* Chat Toggle Button */}
      <button
        className={styles.chatToggle}
        onClick={() => setIsOpen(!isOpen)}
        aria-label={isOpen ? 'Close chat' : 'Open chat assistant'}
      >
        {isOpen ? (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <line x1="18" y1="6" x2="6" y2="18"></line>
            <line x1="6" y1="6" x2="18" y2="18"></line>
          </svg>
        ) : (
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z"></path>
          </svg>
        )}
      </button>

      {/* Chat Window */}
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <div className={styles.chatHeaderContent}>
              <h3>AI Assistant</h3>
              <span>Ask about Physical AI & Robotics</span>
            </div>
            {messages.length > 0 && (
              <button
                className={styles.newChatButton}
                onClick={handleNewChat}
                aria-label="Start new chat"
                title="New Chat"
              >
                <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                  <path d="M12 5v14M5 12h14"></path>
                </svg>
              </button>
            )}
          </div>

          <div className={styles.messagesContainer}>
            {messages.length === 0 && (
              <div className={styles.welcomeMessage}>
                <p>Welcome! Ask me anything about:</p>
                <ul>
                  <li>ROS2 Fundamentals</li>
                  <li>Robot Simulation with Gazebo</li>
                  <li>NVIDIA Isaac Platform</li>
                  <li>Vision-Language-Action Models</li>
                </ul>
              </div>
            )}

            {messages.map((msg, idx) => (
              <div
                key={idx}
                className={`${styles.message} ${
                  msg.role === 'user' ? styles.userMessage : styles.assistantMessage
                }`}
              >
                <div className={styles.messageContent}>
                  {msg.content}
                </div>
                {msg.images && msg.images.length > 0 && (
                  <div className={styles.imagesContainer}>
                    <span className={styles.sourcesLabel}>Related Diagrams:</span>
                    {msg.images.map((image, iidx) => (
                      <a
                        key={iidx}
                        href={image.url}
                        className={styles.imageCard}
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        <img
                          src={image.url}
                          alt={image.alt_text}
                          className={styles.imagePreview}
                        />
                        <span className={styles.imageTitle}>{image.title}</span>
                      </a>
                    ))}
                  </div>
                )}
                {msg.sources && msg.sources.length > 0 && (
                  <div className={styles.sources}>
                    <span className={styles.sourcesLabel}>Sources:</span>
                    {msg.sources.map((source, sidx) => (
                      <a
                        key={sidx}
                        href={source.url}
                        className={styles.sourceLink}
                        target="_blank"
                        rel="noopener noreferrer"
                      >
                        {source.chapter} - {source.section}
                      </a>
                    ))}
                  </div>
                )}
              </div>
            ))}

            {isLoading && (
              <div className={`${styles.message} ${styles.assistantMessage}`}>
                <div className={styles.typing}>
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            )}

            <div ref={messagesEndRef} />
          </div>

          <div className={styles.inputContainer}>
            <input
              ref={inputRef}
              type="text"
              value={input}
              onChange={e => setInput(e.target.value)}
              onKeyPress={handleKeyPress}
              placeholder={selectedText ? "Ask about the selected text..." : "Ask a question..."}
              disabled={isLoading}
              className={styles.input}
            />
            <button
              onClick={sendMessage}
              disabled={isLoading || !input.trim()}
              className={styles.sendButton}
              aria-label="Send message"
            >
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </div>
        </div>
      )}
    </>
  );
}

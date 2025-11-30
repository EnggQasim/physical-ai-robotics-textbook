/**
 * TranslatePopover Component
 *
 * Provides on-demand English to Urdu translation for selected text.
 * Appears as a floating popover near the selected text.
 */
import React, { useState, useEffect, useCallback, useRef } from 'react';
import styles from './styles.module.css';

// API URL configuration
const API_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://mqasim077-physical-ai-textbook-api.hf.space';

interface PopoverPosition {
  top: number;
  left: number;
}

export default function TranslatePopover(): JSX.Element | null {
  const [selectedText, setSelectedText] = useState<string>('');
  const [translation, setTranslation] = useState<string>('');
  const [isLoading, setIsLoading] = useState(false);
  const [isVisible, setIsVisible] = useState(false);
  const [position, setPosition] = useState<PopoverPosition>({ top: 0, left: 0 });
  const [showButton, setShowButton] = useState(false);
  const popoverRef = useRef<HTMLDivElement>(null);
  const buttonRef = useRef<HTMLButtonElement>(null);

  // Handle text selection
  const handleSelectionChange = useCallback(() => {
    const selection = window.getSelection();

    if (!selection || selection.isCollapsed || !selection.toString().trim()) {
      // Delay hiding to allow clicking the translate button
      setTimeout(() => {
        const activeElement = document.activeElement;
        if (activeElement !== buttonRef.current) {
          setShowButton(false);
          setSelectedText('');
        }
      }, 200);
      return;
    }

    const text = selection.toString().trim();

    // Only show for substantial text (not single characters)
    if (text.length < 3 || text.length > 1000) {
      return;
    }

    // Check if selection is within article content (not code blocks or inputs)
    const anchorNode = selection.anchorNode;
    if (!anchorNode) return;

    const parentElement = anchorNode.parentElement;
    if (!parentElement) return;

    // Skip if selecting inside code blocks, inputs, or the popover itself
    if (
      parentElement.closest('pre') ||
      parentElement.closest('code') ||
      parentElement.closest('input') ||
      parentElement.closest('textarea') ||
      parentElement.closest(`.${styles.popover}`) ||
      parentElement.closest(`.${styles.translateButton}`)
    ) {
      return;
    }

    // Get position for the button/popover
    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    setPosition({
      top: rect.top + window.scrollY - 40,
      left: rect.left + window.scrollX + (rect.width / 2)
    });

    setSelectedText(text);
    setShowButton(true);
    setIsVisible(false);
    setTranslation('');
  }, []);

  // Add selection listener
  useEffect(() => {
    document.addEventListener('selectionchange', handleSelectionChange);
    return () => {
      document.removeEventListener('selectionchange', handleSelectionChange);
    };
  }, [handleSelectionChange]);

  // Close popover when clicking outside
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (
        popoverRef.current &&
        !popoverRef.current.contains(event.target as Node) &&
        buttonRef.current &&
        !buttonRef.current.contains(event.target as Node)
      ) {
        setIsVisible(false);
        setShowButton(false);
        setSelectedText('');
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, []);

  // Fetch translation
  const fetchTranslation = async () => {
    if (!selectedText || isLoading) return;

    setIsLoading(true);
    setIsVisible(true);
    setShowButton(false);

    try {
      const response = await fetch(`${API_URL}/api/translation/translate`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          text: selectedText,
          preserve_technical_terms: true,
          context: 'robotics and programming'
        })
      });

      if (!response.ok) {
        throw new Error('Translation failed');
      }

      const data = await response.json();
      setTranslation(data.translation);
    } catch (error) {
      console.error('Translation error:', error);
      setTranslation('ترجمہ دستیاب نہیں (Translation unavailable)');
    } finally {
      setIsLoading(false);
    }
  };

  // Close handler
  const handleClose = () => {
    setIsVisible(false);
    setShowButton(false);
    setSelectedText('');
    setTranslation('');
  };

  if (!showButton && !isVisible) {
    return null;
  }

  return (
    <>
      {/* Translate Button */}
      {showButton && !isVisible && (
        <button
          ref={buttonRef}
          className={styles.translateButton}
          style={{
            top: `${position.top}px`,
            left: `${position.left}px`
          }}
          onClick={fetchTranslation}
          title="Translate to Urdu"
        >
          <svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <path d="M3 5h12M9 3v2m1.048 9.5A18.022 18.022 0 016.412 9m6.088 9h7M11 21l5-10 5 10M12.751 5C11.783 10.77 8.07 15.61 3 18.129" />
          </svg>
          <span>اردو</span>
        </button>
      )}

      {/* Translation Popover */}
      {isVisible && (
        <div
          ref={popoverRef}
          className={styles.popover}
          style={{
            top: `${position.top + 45}px`,
            left: `${position.left}px`
          }}
        >
          <div className={styles.popoverHeader}>
            <span className={styles.popoverTitle}>اردو ترجمہ</span>
            <button className={styles.closeButton} onClick={handleClose}>
              <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <line x1="18" y1="6" x2="6" y2="18" />
                <line x1="6" y1="6" x2="18" y2="18" />
              </svg>
            </button>
          </div>

          <div className={styles.popoverContent}>
            {isLoading ? (
              <div className={styles.loading}>
                <div className={styles.spinner} />
                <span>ترجمہ کیا جا رہا ہے...</span>
              </div>
            ) : (
              <>
                <div className={styles.originalText}>
                  <span className={styles.label}>English:</span>
                  <p>{selectedText}</p>
                </div>
                <div className={styles.translatedText}>
                  <span className={styles.label}>اردو:</span>
                  <p dir="rtl" lang="ur">{translation}</p>
                </div>
              </>
            )}
          </div>
        </div>
      )}
    </>
  );
}

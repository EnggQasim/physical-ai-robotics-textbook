import React, { useState, useCallback, useEffect } from 'react';
import styles from './styles.module.css';

interface DiagramViewerProps {
  concept: string;
  title?: string;
  alt?: string;
  className?: string;
}

interface DiagramData {
  success: boolean;
  url?: string;
  title?: string;
  cached?: boolean;
  error?: string;
}

const API_URL = process.env.NODE_ENV === 'development'
  ? 'http://localhost:8000'
  : 'https://mqasim077-physical-ai-textbook-api.hf.space';

export default function DiagramViewer({
  concept,
  title,
  alt,
  className = '',
}: DiagramViewerProps): JSX.Element {
  const [imageUrl, setImageUrl] = useState<string | null>(null);
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);
  const [isModalOpen, setIsModalOpen] = useState(false);
  const [zoomLevel, setZoomLevel] = useState(1);

  const generateDiagram = useCallback(async () => {
    setIsLoading(true);
    setError(null);

    try {
      const response = await fetch(`${API_URL}/api/diagram/generate`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          concept,
          force_regenerate: false,
        }),
      });

      if (!response.ok) {
        throw new Error('Failed to generate diagram');
      }

      const data: DiagramData = await response.json();

      if (data.success && data.url) {
        setImageUrl(data.url);
      } else {
        setError(data.error || 'Failed to generate diagram');
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'An error occurred');
    } finally {
      setIsLoading(false);
    }
  }, [concept]);

  // Handle keyboard events for modal
  useEffect(() => {
    const handleKeyDown = (e: KeyboardEvent) => {
      if (!isModalOpen) return;

      switch (e.key) {
        case 'Escape':
          setIsModalOpen(false);
          break;
        case '+':
        case '=':
          setZoomLevel((prev) => Math.min(prev + 0.25, 3));
          break;
        case '-':
          setZoomLevel((prev) => Math.max(prev - 0.25, 0.5));
          break;
        case '0':
          setZoomLevel(1);
          break;
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => document.removeEventListener('keydown', handleKeyDown);
  }, [isModalOpen]);

  // Prevent body scroll when modal is open
  useEffect(() => {
    if (isModalOpen) {
      document.body.style.overflow = 'hidden';
    } else {
      document.body.style.overflow = '';
    }
    return () => {
      document.body.style.overflow = '';
    };
  }, [isModalOpen]);

  const handleDownload = async () => {
    if (!imageUrl) return;

    try {
      const response = await fetch(imageUrl);
      const blob = await response.blob();
      const url = window.URL.createObjectURL(blob);
      const link = document.createElement('a');
      link.href = url;
      link.download = `${concept}-diagram.png`;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      window.URL.revokeObjectURL(url);
    } catch (err) {
      console.error('Download failed:', err);
    }
  };

  const openModal = () => {
    setZoomLevel(1);
    setIsModalOpen(true);
  };

  const closeModal = () => {
    setIsModalOpen(false);
  };

  const displayTitle = title || concept.replace(/-/g, ' ').replace(/\b\w/g, (c) => c.toUpperCase());
  const displayAlt = alt || `Diagram illustrating ${displayTitle}`;

  return (
    <div className={`${styles.diagramContainer} ${className}`}>
      {!imageUrl && !isLoading && !error && (
        <button
          className={styles.generateButton}
          onClick={generateDiagram}
          aria-label={`Generate diagram for ${displayTitle}`}
        >
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <rect x="3" y="3" width="18" height="18" rx="2" ry="2" />
            <circle cx="8.5" cy="8.5" r="1.5" />
            <polyline points="21 15 16 10 5 21" />
          </svg>
          <span>Visualize: {displayTitle}</span>
        </button>
      )}

      {isLoading && (
        <div className={styles.loadingContainer}>
          <div className={styles.spinner} />
          <span>Generating diagram...</span>
        </div>
      )}

      {error && (
        <div className={styles.errorContainer}>
          <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
            <circle cx="12" cy="12" r="10" />
            <line x1="12" y1="8" x2="12" y2="12" />
            <line x1="12" y1="16" x2="12.01" y2="16" />
          </svg>
          <span>{error}</span>
          <button className={styles.retryButton} onClick={generateDiagram}>
            Retry
          </button>
        </div>
      )}

      {imageUrl && (
        <div className={styles.imageWrapper}>
          <figure className={styles.figure}>
            <img
              src={imageUrl}
              alt={displayAlt}
              className={styles.diagramImage}
              loading="lazy"
              onClick={openModal}
            />
            <figcaption className={styles.caption}>{displayTitle}</figcaption>
          </figure>
          <div className={styles.toolbar}>
            <button
              className={styles.toolbarButton}
              onClick={openModal}
              aria-label="Expand diagram"
              title="Expand (fullscreen)"
            >
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <polyline points="15 3 21 3 21 9" />
                <polyline points="9 21 3 21 3 15" />
                <line x1="21" y1="3" x2="14" y2="10" />
                <line x1="3" y1="21" x2="10" y2="14" />
              </svg>
            </button>
            <button
              className={styles.toolbarButton}
              onClick={handleDownload}
              aria-label="Download diagram"
              title="Download PNG"
            >
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
                <polyline points="7 10 12 15 17 10" />
                <line x1="12" y1="15" x2="12" y2="3" />
              </svg>
            </button>
            <button
              className={styles.toolbarButton}
              onClick={generateDiagram}
              aria-label="Regenerate diagram"
              title="Regenerate"
            >
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                <polyline points="23 4 23 10 17 10" />
                <polyline points="1 20 1 14 7 14" />
                <path d="M3.51 9a9 9 0 0 1 14.85-3.36L23 10M1 14l4.64 4.36A9 9 0 0 0 20.49 15" />
              </svg>
            </button>
          </div>
        </div>
      )}

      {/* Modal for full-screen view */}
      {isModalOpen && imageUrl && (
        <div className={styles.modalOverlay} onClick={closeModal}>
          <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
            <div className={styles.modalHeader}>
              <h3>{displayTitle}</h3>
              <div className={styles.modalControls}>
                <button
                  className={styles.zoomButton}
                  onClick={() => setZoomLevel((prev) => Math.max(prev - 0.25, 0.5))}
                  aria-label="Zoom out"
                  title="Zoom out (-)"
                >
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <circle cx="11" cy="11" r="8" />
                    <line x1="21" y1="21" x2="16.65" y2="16.65" />
                    <line x1="8" y1="11" x2="14" y2="11" />
                  </svg>
                </button>
                <span className={styles.zoomLevel}>{Math.round(zoomLevel * 100)}%</span>
                <button
                  className={styles.zoomButton}
                  onClick={() => setZoomLevel((prev) => Math.min(prev + 0.25, 3))}
                  aria-label="Zoom in"
                  title="Zoom in (+)"
                >
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <circle cx="11" cy="11" r="8" />
                    <line x1="21" y1="21" x2="16.65" y2="16.65" />
                    <line x1="11" y1="8" x2="11" y2="14" />
                    <line x1="8" y1="11" x2="14" y2="11" />
                  </svg>
                </button>
                <button
                  className={styles.zoomButton}
                  onClick={() => setZoomLevel(1)}
                  aria-label="Reset zoom"
                  title="Reset (0)"
                >
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M3 12a9 9 0 1 0 9-9 9.75 9.75 0 0 0-6.74 2.74L3 8" />
                    <path d="M3 3v5h5" />
                  </svg>
                </button>
                <button
                  className={styles.downloadButton}
                  onClick={handleDownload}
                  aria-label="Download diagram"
                  title="Download"
                >
                  <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4" />
                    <polyline points="7 10 12 15 17 10" />
                    <line x1="12" y1="15" x2="12" y2="3" />
                  </svg>
                </button>
                <button
                  className={styles.closeButton}
                  onClick={closeModal}
                  aria-label="Close modal"
                  title="Close (Esc)"
                >
                  <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
                    <line x1="18" y1="6" x2="6" y2="18" />
                    <line x1="6" y1="6" x2="18" y2="18" />
                  </svg>
                </button>
              </div>
            </div>
            <div className={styles.modalImageWrapper}>
              <img
                src={imageUrl}
                alt={displayAlt}
                className={styles.modalImage}
                style={{ transform: `scale(${zoomLevel})` }}
              />
            </div>
          </div>
        </div>
      )}
    </div>
  );
}

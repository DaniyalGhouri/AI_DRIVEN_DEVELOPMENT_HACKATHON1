// physical-ai-humanoid-robotics-textbook/src/components/HighlightRagButton.tsx

import React, { useState, useEffect, useCallback } from 'react';
import styles from './HighlightRagButton.module.css'; // For styling

interface HighlightRagButtonProps {
  onQuerySent?: (response: any) => void; // Callback after sending query
}

const HighlightRagButton: React.FC<HighlightRagButtonProps> = ({ onQuerySent }) => {
  const [isVisible, setIsVisible] = useState(false);
  const [position, setPosition] = useState({ top: 0, left: 0 });
  const [selectedText, setSelectedText] = useState('');

  const handleTextSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();

    if (text && text.length > 0) {
      setSelectedText(text);
      const range = selection?.getRangeAt(0);
      if (range) {
        const rect = range.getBoundingClientRect();
        // Position the button near the selected text
        setPosition({
          top: window.scrollY + rect.top - 40, // Above the selection
          left: window.scrollX + rect.left + rect.width / 2 - 50, // Centered horizontally
        });
        setIsVisible(true);
      }
    } else {
      setIsVisible(false);
      setSelectedText('');
    }
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, [handleTextSelection]);

  const handleRagQuery = async () => {
    if (!selectedText) return;

    try {
        if (onQuerySent) {
            onQuerySent({ answer: "The backend is currently disconnected.", citations: [] });
        }
    } catch (error) {
      console.error("Error querying RAG for highlighted text:", error);
    } finally {
      setIsVisible(false); // Hide button after query
    }
  };

  if (!isVisible) return null;

  return (
    <button
      className={styles.highlightButton}
      style={{ top: position.top, left: position.left }}
      onClick={handleRagQuery}
    >
      Ask RAG about this
    </button>
  );
};

export default HighlightRagButton;
// physical-ai-humanoid-robotics-textbook/src/components/HighlightRagButton.tsx

import React, { useState, useEffect, useCallback } from 'react';
import styles from './HighlightRagButton.module.css';
import { saveNote } from '../lib/ragClient';

const HighlightRagButton: React.FC = () => {
  const [isVisible, setIsVisible] = useState(false);
  const [position, setPosition] = useState({ top: 0, left: 0 });
  const [selectedText, setSelectedText] = useState('');
  const [saving, setSaving] = useState(false);

  const handleTextSelection = useCallback(() => {
    const selection = window.getSelection();
    const text = selection?.toString().trim();

    if (text && text.length > 0) {
      setSelectedText(text);
      const range = selection?.getRangeAt(0);
      if (range) {
        const rect = range.getBoundingClientRect();
        setPosition({
          top: window.scrollY + rect.top - 60,
          left: window.scrollX + rect.left + rect.width / 2 - 100, 
        });
        setIsVisible(true);
      }
    } else {
      if (!saving) {
        setIsVisible(false);
        setSelectedText('');
      }
    }
  }, [saving]);

  useEffect(() => {
    document.addEventListener('mouseup', handleTextSelection);
    return () => {
      document.removeEventListener('mouseup', handleTextSelection);
    };
  }, [handleTextSelection]);

  const handleAskChatbot = () => {
    if (!selectedText) return;
    const conversationalQuery = `How would you explain this part of the textbook: "${selectedText}"`;
    const event = new CustomEvent('openChatWithContext', { 
      detail: { text: conversationalQuery, type: 'query' } 
    });
    window.dispatchEvent(event);
    setIsVisible(false);
    window.getSelection()?.removeAllRanges();
  };

  const handleSaveNote = async () => {
    if (!selectedText || saving) return;
    setSaving(true);
    try {
      const moduleContext = window.location.pathname.split('/').pop() || 'general';
      await saveNote(selectedText, moduleContext, selectedText);
      
      // Notify user or show success? For now just close menu
      const event = new CustomEvent('noteSaved', { detail: { text: selectedText } });
      window.dispatchEvent(event);
      
      setIsVisible(false);
      window.getSelection()?.removeAllRanges();
    } catch (error: any) {
      console.error("Failed to save note:", error);
      const errorMsg = error.response?.data?.detail || error.message || "Unknown error";
      alert(`Failed to save note: ${errorMsg}`);
    } finally {
      setSaving(false);
    }
  };

  if (!isVisible) return null;

  return (
    <div className={styles.highlightMenu} style={{ top: position.top, left: position.left }}>
      <button
        className={styles.highlightButton}
        onClick={handleAskChatbot}
        title="Ask Chatbot about this"
      >
        Ask Chatbot
      </button>
      <button
        className={styles.noteButton}
        onClick={handleSaveNote}
        disabled={saving}
        title="Save this selection to your notes"
      >
        {saving ? 'Saving...' : '📝 Note'}
      </button>
    </div>
  );
};

export default HighlightRagButton;
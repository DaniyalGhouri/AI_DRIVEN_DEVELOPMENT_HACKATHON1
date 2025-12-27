import React, { useState, useEffect } from 'react';
import { personalizeContent, translateText } from '../lib/ragClient';
import './ChapterTools.css';

interface ChapterToolsProps {
  contentRef?: React.RefObject<HTMLElement>; 
}

const ChapterTools: React.FC<ChapterToolsProps> = () => {
  const [loading, setLoading] = useState(false);
  const [activeAction, setActiveAction] = useState<string | null>(null);
  const [message, setMessage] = useState<string | null>(null);
  const [isLoggedIn, setIsLoggedIn] = useState(false);

  useEffect(() => {
    const checkLogin = () => {
      const token = localStorage.getItem('access_token');
      setIsLoggedIn(!!token);
    };
    
    checkLogin();
    window.addEventListener('storage', checkLogin);
    return () => window.removeEventListener('storage', checkLogin);
  }, []);

  const getContent = (): HTMLElement | null => {
    const article = document.querySelector('article');
    if (!article) return null;
    return article;
  };

  const handlePersonalize = async () => {
    const article = getContent();
    if (!article) return;

    setLoading(true);
    setActiveAction('personalize');
    setMessage('Adapting content to your profile...');

    try {
      const originalText = article.innerText;
      const response = await personalizeContent(originalText, 'current-chapter');
      
      const newContentDiv = document.createElement('div');
      newContentDiv.className = 'personalized-content theme-doc-markdown markdown';
      newContentDiv.innerHTML = `<h3>🧠 Personalized for You</h3><p>${response.personalized_text.replace(/\n/g, '<br/>')}</p>`;
      
      article.style.display = 'none';
      article.parentNode?.insertBefore(newContentDiv, article);
      
      const resetBtn = document.createElement('button');
      resetBtn.innerText = 'Restore Original';
      resetBtn.className = 'tool-reset-btn';
      resetBtn.onclick = () => {
          newContentDiv.remove();
          article.style.display = 'block';
          setActiveAction(null);
          setMessage(null);
      };
      newContentDiv.appendChild(resetBtn);

      setMessage(null);
    } catch (error) {
      console.error(error);
      setMessage('Failed to personalize. Is backend running?');
    } finally {
      setLoading(false);
    }
  };

  const handleTranslate = async () => {
    const article = getContent();
    if (!article) return;

    setLoading(true);
    setActiveAction('translate');
    setMessage('Translating to Urdu...');

    try {
      const originalText = article.innerText;
      const response = await translateText(originalText, 'Urdu');
      
      const newContentDiv = document.createElement('div');
      newContentDiv.className = 'translated-content theme-doc-markdown markdown';
      newContentDiv.style.direction = 'rtl'; 
      newContentDiv.innerHTML = `<h3>🌐 Urdu Translation</h3><p>${response.translated_text.replace(/\n/g, '<br/>')}</p>`;
      
      article.style.display = 'none';
      article.parentNode?.insertBefore(newContentDiv, article);
      
      const resetBtn = document.createElement('button');
      resetBtn.innerText = 'Restore Original';
      resetBtn.className = 'tool-reset-btn';
      resetBtn.onclick = () => {
          newContentDiv.remove();
          article.style.display = 'block';
          setActiveAction(null);
          setMessage(null);
      };
      newContentDiv.appendChild(resetBtn);

      setMessage(null);
    } catch (error) {
      console.error(error);
      setMessage('Failed to translate. Is backend running?');
    } finally {
      setLoading(false);
    }
  };

  const handleLogout = () => {
    localStorage.removeItem('access_token');
    window.location.reload(); 
  };

  if (!isLoggedIn) return null;

  return (
    <div className="chapter-tools-dock">
      <div className="dock-actions">
        <button 
          onClick={handlePersonalize} 
          disabled={loading}
          className={`dock-btn ${activeAction === 'personalize' ? 'active' : ''}`}
          title="Rewrite content based on your background"
        >
          {loading && activeAction === 'personalize' ? '⚙️' : '🧠'} <span>Study Mode</span>
        </button>
        
        <button 
          onClick={handleTranslate} 
          disabled={loading}
          className={`dock-btn ${activeAction === 'translate' ? 'active' : ''}`}
          title="Translate page to Urdu"
        >
          {loading && activeAction === 'translate' ? '⚙️' : '🌐'} <span>Urdu</span>
        </button>
      </div>

      {message && <div className="dock-message">{message}</div>}

      <div className="dock-divider"></div>
      
      <button onClick={handleLogout} className="dock-btn logout" title="Logout">
        🚪 <span>Logout</span>
      </button>
    </div>
  );
};

export default ChapterTools;
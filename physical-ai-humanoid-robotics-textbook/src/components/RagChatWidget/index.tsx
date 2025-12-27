// physical-ai-humanoid-robotics-textbook/src/components/RagChatWidget/index.tsx

import React, { useState, useEffect } from 'react';
import styles from './styles.module.css';
import ChatPanel from '@site/src/components/ChatPanel';

const RagChatWidget: React.FC = () => {
  const [isChatPanelOpen, setIsChatPanelOpen] = useState(false);
  const [isLoggedIn, setIsLoggedIn] = useState(false);

  useEffect(() => {
    const checkLogin = () => {
      const token = localStorage.getItem('access_token');
      setIsLoggedIn(!!token);
    };

    checkLogin();
    
    // Listen for custom events or storage changes
    window.addEventListener('storage', checkLogin);
    
    // Custom event listener for 'openChatWithContext' also acts as a login validator
    const handleOpenChat = () => {
      checkLogin();
      setIsChatPanelOpen(true);
    };

    window.addEventListener('openChatWithContext', handleOpenChat);
    return () => {
      window.removeEventListener('storage', checkLogin);
      window.removeEventListener('openChatWithContext', handleOpenChat);
    };
  }, []);

  const toggleChatPanel = () => {
    setIsChatPanelOpen(!isChatPanelOpen);
  };

  // Only show the chat widget if the user is logged in
  if (!isLoggedIn) return null;

  return (
    <>
      <div className={styles.chatWidgetContainer}>
        <button className={styles.chatToggleButton} onClick={toggleChatPanel}>
          {isChatPanelOpen ? 'Close Chat' : 'Chat with Textbook'}
        </button>
      </div>
      <ChatPanel isOpen={isChatPanelOpen} onClose={() => setIsChatPanelOpen(false)} />
    </>
  );
};

export default RagChatWidget;

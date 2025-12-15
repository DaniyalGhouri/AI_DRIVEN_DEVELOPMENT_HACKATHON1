// physical-ai-humanoid-robotics-textbook/src/components/RagChatWidget/index.tsx

import React, { useState } from 'react';
import styles from './styles.module.css';
import ChatPanel from '@site/src/components/ChatPanel'; // Import ChatPanel

const RagChatWidget: React.FC = () => {
  const [isChatPanelOpen, setIsChatPanelOpen] = useState(false);

  const toggleChatPanel = () => {
    setIsChatPanelOpen(!isChatPanelOpen);
  };

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

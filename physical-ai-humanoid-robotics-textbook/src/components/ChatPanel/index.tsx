import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import { queryRAG, translateText, getRecommendations } from '@site/src/lib/ragClient'; // Import getRecommendations

interface ChatPanelProps {
  isOpen: boolean;
  onClose: () => void;
}

interface ChatMessage {
  type: 'user' | 'bot';
  text: string;
  translatedText?: string;
  isLoading?: boolean; // Added for loading state
}

interface UserNote {
  note_id: string;
  note_content: string;
  created_at: string;
}

interface ChatSession {
  session_id: string;
  chat_history: ChatMessage[];
  created_at: string;
}

const ChatPanel: React.FC<ChatPanelProps> = ({ isOpen, onClose }) => {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [input, setInput] = useState<string>('');
  const [userNotes, setUserNotes] = useState<UserNote[]>([]);
  const [chatSessions, setChatSessions] = useState<ChatSession[]>([]);
  const [activeTab, setActiveTab] = useState<'chat' | 'notes' | 'recommendations'>('chat');
  const [isUrduTranslationEnabled, setIsUrduTranslationEnabled] = useState<boolean>(false);
  const [recommendations, setRecommendations] = useState<string>('');
  const [isLoading, setIsLoading] = useState<boolean>(false); // Global loading state
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(scrollToBottom, [messages]);

  useEffect(() => {
    if (isOpen) {
      const fetchUserData = async () => {
        const token = localStorage.getItem('access_token'); 
        if (!token) {
          console.warn("No access token found for fetching user data.");
          // In a real app, you might redirect to login or show an error
          return;
        }

        try {
          // Mock data for development
          setUserNotes([
            { note_id: '1', note_content: 'ROS2 is great for robotics.', created_at: '2023-01-01' },
            { note_id: '2', note_content: 'FastAPI is a Python web framework.', created_at: '2023-01-05' },
          ]);
          setChatSessions([
            { session_id: 's1', chat_history: [{ type: 'user', text: 'What is ROS2?' }, { type: 'bot', text: 'ROS2 is a flexible framework for writing robot software.' }], created_at: '2023-01-01' },
          ]);

        } catch (error) {
          console.error("Failed to fetch user data:", error);
        }
      };
      fetchUserData();
    }
  }, [isOpen]);

  const handleSendMessage = async () => {
    if (input.trim() && !isLoading) {
      const userMessage: ChatMessage = { type: 'user', text: input };
      const newMessages = [...messages, userMessage];
      setMessages(newMessages);
      const userQuery = input;
      setInput('');
      setIsLoading(true);

      try {
        const response = await queryRAG(userQuery);
        let botMessage: ChatMessage = { type: 'bot', text: response.answer };

        if (isUrduTranslationEnabled) {
          const translatedResponse = await translateText(response.answer, 'ur');
          botMessage.translatedText = translatedResponse.translated_text;
        }
        setMessages((prevMessages) => [...prevMessages, botMessage]);
      } catch (error) {
        setMessages((prevMessages) => [...prevMessages, { type: 'bot', text: "Error: Could not get a response." }]);
        console.error("Error sending message:", error);
      } finally {
        setIsLoading(false);
      }
    }
  };

  const fetchRecommendations = async () => {
    if (isLoading) return;
    setIsLoading(true);
    try {
      const response = await getRecommendations();
      setRecommendations(response.recommendations);
    } catch (error) {
      console.error("Failed to fetch recommendations:", error);
      setRecommendations("Failed to load recommendations.");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`${styles.chatPanelContainer} ${isOpen ? styles.isOpen : ''}`}>
      <div className={styles.chatHeader}>
        <h3 className={styles.chatTitle}>Textbook Chatbot</h3>
        <div className={styles.headerActions}>
          <button 
            className={styles.translateToggleButton} 
            onClick={() => setIsUrduTranslationEnabled(!isUrduTranslationEnabled)}
            aria-label={isUrduTranslationEnabled ? 'Disable Urdu Translation' : 'Enable Urdu Translation'}
            title={isUrduTranslationEnabled ? 'Disable Urdu Translation' : 'Enable Urdu Translation'}
          >
            {isUrduTranslationEnabled ? 'Urdu: On' : 'Urdu: Off'}
          </button>
          <button className={styles.closeButton} onClick={onClose} aria-label="Close Chat" title="Close Chat">
            &times;
          </button>
        </div>
      </div>
      <div className={styles.tabs}>
        <button 
          className={`${styles.tabButton} ${activeTab === 'chat' ? styles.active : ''}`}
          onClick={() => setActiveTab('chat')}
          aria-selected={activeTab === 'chat'}
          role="tab"
        >
          Chat
        </button>
        <button 
          className={`${styles.tabButton} ${activeTab === 'notes' ? styles.active : ''}`}
          onClick={() => setActiveTab('notes')}
          aria-selected={activeTab === 'notes'}
          role="tab"
        >
          My Notes
        </button>
        <button 
          className={`${styles.tabButton} ${activeTab === 'recommendations' ? styles.active : ''}`}
          onClick={() => setActiveTab('recommendations')}
          aria-selected={activeTab === 'recommendations'}
          role="tab"
        >
          Recommendations
        </button>
      </div>
      <div className={styles.chatContent}>
        {activeTab === 'chat' ? (
          <>
            <div className={styles.chatMessagesArea}>
              {messages.map((msg, index) => (
                <div key={index} className={`${styles.message} ${styles[msg.type]}`}>
                  {msg.isLoading ? (
                    <p>Thinking...</p>
                  ) : (
                    <>
                      <p>{msg.text}</p>
                      {isUrduTranslationEnabled && msg.translatedText && msg.type === 'bot' && (
                        <p className={styles.translatedText}>({msg.translatedText})</p>
                      )}
                    </>
                  )}
                </div>
              ))}
              {/* Display previous chat sessions if any, or merge them into current chat */}
              {chatSessions.map((session, sIndex) => (
                <div key={`session-${sIndex}`} className={styles.pastSession}>
                  <h4>Past Session ({new Date(session.created_at).toLocaleDateString()})</h4>
                  {session.chat_history.map((msg, mIndex) => (
                     <div key={`session-${sIndex}-msg-${mIndex}`} className={`${styles.message} ${styles[msg.type]}`}>
                       <p>{msg.text}</p>
                       {isUrduTranslationEnabled && msg.translatedText && msg.type === 'bot' && (
                         <p className={styles.translatedText}>({msg.translatedText})</p>
                       )}
                     </div>
                  ))}
                </div>
              ))}
              <div ref={messagesEndRef} /> {/* Scroll to this div */}
            </div>
            <div className={styles.chatInputArea}>
              <input 
                type="text" 
                placeholder="Type your question here..." 
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => { if (e.key === 'Enter') handleSendMessage(); }}
                disabled={isLoading}
                aria-label="Chat input"
              />
              <button onClick={handleSendMessage} disabled={isLoading} aria-label="Send message">
                Send
              </button>
            </div>
          </>
        ) : activeTab === 'notes' ? (
          <div className={styles.userNotesArea}>
            <h4 className={styles.sectionTitle}>Your Notes</h4>
            {userNotes.length > 0 ? (
              userNotes.map((note) => (
                <div key={note.note_id} className={styles.noteItem}>
                  <p>{note.note_content}</p>
                  <small>{new Date(note.created_at).toLocaleDateString()}</small>
                </div>
              ))
            ) : (
              <p>No notes found.</p>
            )}
          </div>
        ) : (
          <div className={styles.recommendationsPanel}>
            <h4 className={styles.sectionTitle}>Personalized Recommendations</h4>
            <button onClick={fetchRecommendations} disabled={isLoading}>
              {isLoading ? 'Loading...' : 'Get Recommendations'}
            </button>
            <div className={styles.recommendationsDisplay}>
              {recommendations ? <p>{recommendations}</p> : <p>Click "Get Recommendations" to see suggestions based on your interactions.</p>}
            </div>
          </div>
        )}
      </div>
    </div>
  );
};

export default ChatPanel;

import React, { useState, useEffect, useRef } from 'react';
import styles from './styles.module.css';
import { queryRAG, translateText, getRecommendations, getUserNotes, getUserChatSessions, saveNote } from '@site/src/lib/ragClient';

interface ChatPanelProps {
  isOpen: boolean;
  onClose: () => void;
}

interface ChatMessage {
  type: 'user' | 'bot';
  text: string;
  translatedText?: string;
  isLoading?: boolean;
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
  const [isLoading, setIsLoading] = useState<boolean>(false);
  const [newNote, setNewNote] = useState('');
  const [isSavingNote, setIsSavingNote] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  };

  useEffect(scrollToBottom, [messages]);

  const fetchUserData = async () => {
    const token = localStorage.getItem('access_token');
    if (!token) return;

    try {
      const [notesRes, sessionsRes] = await Promise.allSettled([
        getUserNotes(),
        getUserChatSessions()
      ]);

      if (notesRes.status === 'fulfilled') {
        setUserNotes(notesRes.value);
      }
      if (sessionsRes.status === 'fulfilled') {
        setChatSessions(sessionsRes.value);
      }
    } catch (error) {
      console.error("Failed to fetch user data:", error);
    }
  };

  useEffect(() => {
    if (isOpen) {
      fetchUserData();
    }
  }, [isOpen]);

  const handleSendMessage = async (textOverride?: string) => {
    const textToSend = typeof textOverride === 'string' ? textOverride : input;
    
    if (textToSend.trim() && !isLoading) {
      const userMessage: ChatMessage = { type: 'user', text: textToSend };
      setMessages(prev => [...prev, userMessage]);
      setInput(''); 
      setIsLoading(true);
      setActiveTab('chat'); 

      try {
        const response = await queryRAG(textToSend);
        let botMessage: ChatMessage = { type: 'bot', text: response.answer };

        if (isUrduTranslationEnabled) {
          const translatedResponse = await translateText(response.answer, 'ur');
          botMessage.translatedText = translatedResponse.translated_text;
        }
        setMessages((prevMessages) => [...prevMessages, botMessage]);
      } catch (error) {
        setMessages((prevMessages) => [...prevMessages, { type: 'bot', text: "Error: Could not get a response." }]);
      } finally {
        setIsLoading(false);
      }
    }
  };

  const handleAddManualNote = async () => {
    if (!newNote.trim() || isSavingNote) return;
    setIsSavingNote(true);
    try {
      await saveNote(newNote);
      setNewNote('');
      fetchUserData(); // Refresh notes
    } catch (error: any) {
      console.error("Failed to save note:", error);
      const errorMsg = error.response?.data?.detail || error.message || "Unknown error";
      alert(`Failed to save note: ${errorMsg}`);
    } finally {
      setIsSavingNote(false);
    }
  };

  // Listen for custom events
  useEffect(() => {
    const handleContextQuery = (e: Event) => {
      const customEvent = e as CustomEvent;
      if (customEvent.detail && customEvent.detail.text) {
        const text = customEvent.detail.text;
        setInput(text);
        handleSendMessage(text);
      }
    };

    const handleNoteSaved = () => {
      fetchUserData(); // Refresh list when note is saved via highlight
    };

    window.addEventListener('openChatWithContext', handleContextQuery);
    window.addEventListener('noteSaved', handleNoteSaved);
    
    return () => {
      window.removeEventListener('openChatWithContext', handleContextQuery);
      window.removeEventListener('noteSaved', handleNoteSaved);
    };
  }, [isLoading, isUrduTranslationEnabled]);

  const handleNewChat = () => {
    setMessages([]);
  };

  const fetchRecommendations = async () => {
    if (isLoading) return;
    setIsLoading(true);
    try {
      const response = await getRecommendations();
      setRecommendations(response.recommendations);
    } catch (error) {
      setRecommendations("Failed to load recommendations.");
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <div className={`${styles.chatPanelContainer} ${isOpen ? styles.isOpen : ''}`}>
      <div className={styles.chatHeader}>
        <h3 className={styles.chatTitle}>AI Assistant</h3>
        <div className={styles.headerActions}>
          <button 
            className={styles.translateToggleButton} 
            onClick={() => setIsUrduTranslationEnabled(!isUrduTranslationEnabled)}
            title={isUrduTranslationEnabled ? 'Disable Urdu' : 'Enable Urdu'}
          >
            {isUrduTranslationEnabled ? 'Urdu On' : 'Urdu Off'}
          </button>
          <button className={styles.iconButton} onClick={handleNewChat} title="New Chat">
            +
          </button>
          <button className={styles.iconButton} onClick={() => { localStorage.removeItem('access_token'); window.location.reload(); }} title="Logout">
            🚪
          </button>
          <button className={styles.iconButton} onClick={onClose} title="Close">
            &times;
          </button>
        </div>
      </div>
      
      <div className={styles.tabs}>
        <button 
          className={`${styles.tabButton} ${activeTab === 'chat' ? styles.active : ''}`}
          onClick={() => setActiveTab('chat')}
        >
          Chat
        </button>
        <button 
          className={`${styles.tabButton} ${activeTab === 'notes' ? styles.active : ''}`}
          onClick={() => setActiveTab('notes')}
        >
          Notes
        </button>
        <button 
          className={`${styles.tabButton} ${activeTab === 'recommendations' ? styles.active : ''}`}
          onClick={() => setActiveTab('recommendations')}
        >
          For You
        </button>
      </div>

      <div className={styles.chatContent}>
        {activeTab === 'chat' ? (
          <>
            <div className={styles.chatMessagesArea}>
              {chatSessions.length > 0 && messages.length === 0 && (
                <div style={{ textAlign: 'center', color: '#888', marginTop: '20px' }}>
                  <p>Welcome back! Check your past sessions below or start a new chat.</p>
                </div>
              )}
              
              {messages.map((msg, index) => (
                <div key={index} className={`${styles.message} ${styles[msg.type]}`}>
                  {msg.isLoading ? (
                    <p>Thinking...</p>
                  ) : (
                    <>
                      <p>{msg.text}</p>
                      {msg.translatedText && (
                        <p className={styles.translatedText}>{msg.translatedText}</p>
                      )}
                    </>
                  )}
                </div>
              ))}
              
              {messages.length === 0 && chatSessions.map((session, sIndex) => (
                <div key={`session-${sIndex}`} style={{ margin: '20px 0', borderTop: '1px solid #eee', paddingTop: '10px' }}>
                  <small style={{ color: '#999', display: 'block', textAlign: 'center' }}>
                    Session from {new Date(session.created_at).toLocaleDateString()}
                  </small>
                  {session.chat_history.map((msg, mIndex) => (
                     <div key={`session-${sIndex}-msg-${mIndex}`} className={`${styles.message} ${styles[msg.type]}`} style={{ opacity: 0.8 }}>
                       <p>{msg.text}</p>
                     </div>
                  ))}
                </div>
              ))}
              <div ref={messagesEndRef} />
            </div>
            <div className={styles.chatInputArea}>
              <input 
                type="text" 
                placeholder="Ask anything..." 
                value={input}
                onChange={(e) => setInput(e.target.value)}
                onKeyPress={(e) => { if (e.key === 'Enter') handleSendMessage(); }}
                disabled={isLoading}
              />
              <button onClick={() => handleSendMessage()} disabled={isLoading || !input.trim()}>
                Send
              </button>
            </div>
          </>
        ) : activeTab === 'notes' ? (
          <div className={styles.userNotesArea}>
            <div className={styles.addNoteSection}>
              <textarea 
                placeholder="Type a new note here..."
                value={newNote}
                onChange={(e) => setNewNote(e.target.value)}
                className={styles.noteInput}
              />
              <button 
                onClick={handleAddManualNote} 
                disabled={isSavingNote || !newNote.trim()}
                className={styles.addNoteBtn}
              >
                {isSavingNote ? 'Saving...' : 'Add Note'}
              </button>
            </div>
            
            <h4 className={styles.sectionTitle}>Your Saved Notes</h4>
            {userNotes.length > 0 ? (
              userNotes.map((note) => (
                <div key={note.note_id} className={styles.noteItem}>
                  <p>{note.note_content}</p>
                  <small style={{ color: '#999' }}>{new Date(note.created_at).toLocaleDateString()}</small>
                </div>
              ))
            ) : (
              <p style={{ textAlign: 'center', color: '#888' }}>No notes found.</p>
            )}
          </div>
        ) : (
          <div className={styles.recommendationsPanel}>
            <h4 className={styles.sectionTitle}>Recommended Content</h4>
            <button onClick={fetchRecommendations} disabled={isLoading}>
              {isLoading ? 'Generating...' : 'Get New Recommendations'}
            </button>
            {recommendations && (
              <div className={styles.recommendationsDisplay}>
                <p>{recommendations}</p>
              </div>
            )}
          </div>
        )}
      </div>
    </div>
  );
};

export default ChatPanel;
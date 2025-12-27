import React, { useState, useEffect } from 'react';
import RagChatWidget from '@site/src/components/RagChatWidget';
import HighlightRagButton from '@site/src/components/HighlightRagButton';
import AuthModal from '@site/src/components/AuthModal';
import ChapterTools from '@site/src/components/ChapterTools';
import { useLocation } from '@docusaurus/router';

// Helper to check if we are on a doc page
const isDocPage = (pathname) => {
  return pathname.startsWith('/docs/') || pathname.startsWith('/modules/');
};

function Root({ children }) {
  const [isAuthModalOpen, setIsAuthModalOpen] = useState(false);
  const [isLoggedIn, setIsLoggedIn] = useState(false);
  const location = useLocation();

  useEffect(() => {
    const token = localStorage.getItem('access_token');
    setIsLoggedIn(!!token);
  }, []);

  const handleLoginSuccess = () => {
    setIsLoggedIn(true);
  };

  // Inject ChapterTools into the content if we are on a doc page
  // Since we can't easily modify the children structure deep down from Root without swizzling DocItem,
  // we will rely on ChapterTools logic to find the article and insert itself or be placed via portal if possible.
  // Actually, standard Swizzling of DocItem/Content is better for placement.
  // But for "Root" approach, we can place it fixed or try to render it. 
  // Wait, I can't inject *inside* the markdown content from here easily. 
  // However, I can render it *above* the children if the children is the page layout.
  
  // A better hack for Root: The `children` is the whole app layout. 
  // We can't easily inject "at the start of the chapter" (inside the layout) from Root.
  // The user prompt said "pressing a button at the start of each chapter".
  // The `ChapterTools` component I wrote tries to find `<article>` and operate on it.
  // Let's render `ChapterTools` here, and it will try to find the article.
  // But `ChapterTools` needs to be inside the flow to look good.
  
  // Alternative: We can add a "Login" button to the Navbar via docusaurus.config.js or here.
  // For now, let's put a floating "Login" button if not logged in, or rely on the ChatWidget to prompt login?
  // Let's add a simple floating auth button for this hackathon requirement.

  return (
    <>
      {children}
      
      {/* Tools that overlay or float */}
      <RagChatWidget />
      <HighlightRagButton />
      
      {/* Auth Modal */}
      <AuthModal 
        isOpen={isAuthModalOpen} 
        onClose={() => setIsAuthModalOpen(false)} 
        onLoginSuccess={handleLoginSuccess}
      />

      {/* Login Trigger (if not logged in) */}
      {!isLoggedIn && (
        <button 
          style={{
            position: 'fixed',
            bottom: '20px',
            left: '20px',
            zIndex: 100,
            padding: '10px 20px',
            borderRadius: '20px',
            border: 'none',
            backgroundColor: '#25c2a0',
            color: 'white',
            fontWeight: 'bold',
            cursor: 'pointer',
            boxShadow: '0 2px 10px rgba(0,0,0,0.2)'
          }}
          onClick={() => setIsAuthModalOpen(true)}
        >
          Login / Sign Up
        </button>
      )}

      {/* Chapter Tools - Top Toolbar Style */}
      {isLoggedIn && isDocPage(location.pathname) && (
        <div style={{ position: 'fixed', top: '60px', left: 0, right: 0, zIndex: 95 }}>
           <ChapterTools key={location.pathname} />
        </div>
      )}
    </>
  );
}

export default Root;
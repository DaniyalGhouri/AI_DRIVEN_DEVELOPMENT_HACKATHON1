// physical-ai-humanoid-robotics-textbook/src/theme/Root.js

import React from 'react';
import RagChatWidget from '@site/src/components/RagChatWidget';
import HighlightRagButton from '@site/src/components/HighlightRagButton'; // Import the new component

function Root({ children }) {
  return (
    <>
      {children}
      <RagChatWidget />
      <HighlightRagButton /> {/* Add the HighlightRagButton here */}
    </>
  );
}

export default Root;

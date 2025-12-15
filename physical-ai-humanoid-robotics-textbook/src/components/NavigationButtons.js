import React from 'react';
import Link from '@docusaurus/Link';

export default function NavigationButtons({ prev, next, module }) {
  return (
    <div className="navigation-buttons">
      {prev && (
        <Link className="button button--secondary" to={prev.url}>
          ← Previous: {prev.label}
        </Link>
      )}
      
      {module && (
        <Link className="button button--primary" to={module.url}>
          ↑ Back to Module
        </Link>
      )}
      
      {next && (
        <Link className="button button--secondary" to={next.url}>
          Next: {next.label} →
        </Link>
      )}
    </div>
  );
}
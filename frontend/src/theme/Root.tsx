import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';
import { AuthProvider } from '@site/src/components/Auth';
import { PersonalizationProvider, OnboardingModal } from '@site/src/components/Personalization';

// Wrap the Root component to include ChatWidget, AuthProvider, and PersonalizationProvider on all pages
export default function Root({children}: {children: React.ReactNode}): JSX.Element {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        {children}
        <ChatWidget />
        <OnboardingModal />
      </PersonalizationProvider>
    </AuthProvider>
  );
}

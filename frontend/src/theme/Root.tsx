import React from 'react';
import ChatWidget from '@site/src/components/ChatWidget';
import { AuthProvider } from '@site/src/components/Auth';
import { PersonalizationProvider, OnboardingModal } from '@site/src/components/Personalization';
import TranslatePopover from '@site/src/components/TranslatePopover';

// Wrap the Root component to include ChatWidget, AuthProvider, PersonalizationProvider, and TranslatePopover on all pages
export default function Root({children}: {children: React.ReactNode}): JSX.Element {
  return (
    <AuthProvider>
      <PersonalizationProvider>
        {children}
        <ChatWidget />
        <OnboardingModal />
        <TranslatePopover />
      </PersonalizationProvider>
    </AuthProvider>
  );
}

import React, { useState } from 'react';
import BrowserOnly from '@docusaurus/BrowserOnly';

const VoiceCommandDemo = () => {
  const [isListening, setIsListening] = useState(false);
  const [transcript, setTranscript] = useState('');

  const startListening = () => {
    setIsListening(true);
    // This is a simulation - in a real implementation, you would use the Web Speech API
    setTimeout(() => {
      setTranscript('Move forward 2 meters');
      setIsListening(false);
    }, 2000);
  };

  return (
    <div style={{ padding: '20px', border: '1px solid #ccc', borderRadius: '8px', margin: '20px 0' }}>
      <h4>Voice Command Demo</h4>
      <p>Click the button below to simulate voice recognition:</p>
      <button
        onClick={startListening}
        disabled={isListening}
        style={{
          padding: '10px 15px',
          backgroundColor: isListening ? '#ccc' : '#007cba',
          color: 'white',
          border: 'none',
          borderRadius: '4px',
          cursor: isListening ? 'not-allowed' : 'pointer'
        }}
      >
        {isListening ? 'Listening...' : 'Start Voice Recognition'}
      </button>

      {transcript && (
        <div style={{ marginTop: '15px', padding: '10px', backgroundColor: '#f0f8ff', borderRadius: '4px' }}>
          <strong>Recognized Command:</strong> {transcript}
        </div>
      )}

      <div style={{ marginTop: '15px', fontSize: '0.9em', color: '#666' }}>
        <p><strong>Note:</strong> This is a simulation. In a real implementation, this would connect to OpenAI Whisper API.</p>
      </div>
    </div>
  );
};

export default VoiceCommandDemo;
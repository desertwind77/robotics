import speech_recognition as sr

class VoiceRecognizer( ThreadBase ):
    '''
    Perform voice recognition via google service
    '''
    def __init__( self, name, callback ):
        super( VoiceRecognizer, self ).__init__( name )
        self.callback = callback
        self.speech_recognier = None
        self.output_filename = 'speech_output.wav'

    def run( self ):
        self.speech_recognier = sr.Recognizer()

        while not self.isExiting.is_set():
            with sr.Microphone() as source:
                self.speech_recognier.adjust_for_ambient_noise( source )

                log( voiceRecognizer, Level_Debug, 'Waiting for command...' )
                audio = self.speech_recognier.listen( source )

                if isDebug( voiceCommand ):
                    # TODO : need file management here
                    log( voiceRecognizer, Level_Debug,
                            'Saving audio to {}'.format( self.output_filename ) )
                    with open( self.output_filename, 'wb' ) as f:
                        f.write( audio.get_wav_data() )

                command = None
                try:
                    log( voiceRecognizer, Level_Debug, 'Analyzing...' )
                    text = self.speech_recognier.recognize_google( audio,
                            language = 'en-Us' )
                    log( voiceRecognizer, Level_Debug,
                            'You said : {}'.format( text ) )
                    command = text
                except sr.UnknownValueError:
                    log( voiceRecognizer, Level_Debug,
                            'Unable to understand the commmand' )
                except sr.RequestError as e:
                    errorMsg = 'Unable to request results from '
                    errorMsg += 'Google Speech Recognition : {0}'.format( e )
                    log( voiceRecognizer, Level_Error, errorMsg )

                if command:
                    self.callback( command )


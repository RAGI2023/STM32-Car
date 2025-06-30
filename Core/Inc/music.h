
#ifndef MUSIC_H
#define MUSIC_H

#ifdef __cplusplus
extern "C" {
#endif

#define MUSIC_LENGTH 200
	
typedef enum
{
  MUSIC_STOP        		= 0x00U,    
  MUSIC_IDLE        		,    
  MUSIC_START     			,    
} Music_State_TypeDef;

void MusicProcess( void );

extern __IO Music_State_TypeDef  g_music_state  ;
#ifdef __cplusplus
}
#endif

#endif /* MUSIC_H */

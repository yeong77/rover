#include <Keyboard.h>

static struct termios initial_settings, new_settings;
static int peek_character = -1;
bool isTerminalChanged = false;

// Set teminal mode to non canonical mode
void init_keyboard()
{
    isTerminalChanged = true;
    tcgetattr(0,&initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~ICANON; // ICANON : Canonical input (erase and kill processing)
    new_settings.c_lflag &= ~ECHO;  // ECHO : Enable echo / 에코 해제
    new_settings.c_cc[VMIN] = 1;    //c_cc : control chars
    new_settings.c_cc[VTIME] = 0;   //VMIN : read() 함수가 return 되기 위한 최소 char 개수 
                                    //VTIME : 타이머 사용 (0 :사용X) (>0 :Time-out 사용 TIME*0.1 sec)
                                        //Time-out이 일어나기 전에 한 문자라도 들어오면 read() 리턴 가능
    tcsetattr(0, TCSANOW, &new_settings);
}

// Restore teminal setting to original
void close_keyboard()
{
    if (isTerminalChanged)
        tcsetattr(0, TCSANOW, &initial_settings);
}


// Check terminal input buffer
int _kbhit()
{
    unsigned char ch;
    int nread;
 
    if (peek_character != -1) return 1; //there is an char
    new_settings.c_cc[VMIN]=0;
    tcsetattr(0, TCSANOW, &new_settings);

    //read(int fild, void *buf, size_t nbytes)
    //fild : 읽을 파일의 파일 디스크럽터, buf : 읽어들인 데이터를 저장할 버퍼, nbytes : 읽어들일 데이터의 최대 길이 (< buf)
    //read() : 읽어들인 데이터의 길이, 데이터가 없으면 -1 return
    nread = read(0,&ch,1);

    new_settings.c_cc[VMIN]=1;
    tcsetattr(0, TCSANOW, &new_settings);
    if(nread == 1)
    {
        peek_character = ch;    //저장된 char 복사
        return 1;
    }
    return 0;
}

// Return input char
int _getch()
{
    char ch;
 
    if(peek_character != -1)
    {
        ch = peek_character;
        peek_character = -1;
        return ch;
    }
    read(0,&ch,1);  //if 문을 지나옴 -> 저장된 캐릭터가 없다는 의미
                    //-> ch = ' ' or 0(아마도)
    return ch;
}

// Display char in terminal
int _putch(int c) {
    putchar(c);
    fflush(stdout);
    return c;
}
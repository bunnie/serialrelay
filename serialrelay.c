#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h> // needed for memset
#include <dirent.h>
#include <signal.h>
#include <sys/file.h>
#include <errno.h>

#define TTYDEV "/home/www-data/web2py/scripts/ttyUSB0"

struct termios old_stdout;
struct termios old_stdin;
int tty_fd;

void sigcatch(int sig)
{
  tcsetattr(STDOUT_FILENO, TCSAFLUSH, &old_stdout);
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &old_stdin);
  
  if( flock(tty_fd, LOCK_UN) == -1 ) {
    fprintf( stderr,  "flock() unlock failed with %s (%i)\n", strerror(errno), errno );
  }
  close(tty_fd);

  exit(0);
}

int main(int argc,char** argv)
{
  struct termios tio;
  struct termios new_stdout;
  struct termios new_stdin;
  fd_set rdset;
 
  unsigned char c='D';

  struct dirent **namelist;
  int n, m;
  char dirstr[1024];

  fd_set serialfds;
  struct timeval tv;
  int retval;

  char bfr[512];
  int count;

  char *tty_devname;

  /* Catch the most popular signals. */
  if((int) signal(SIGINT,sigcatch) < 0)
    {
      perror("signal");
      exit(1);
    }
  if((int)signal(SIGQUIT,sigcatch) < 0)
    {
      perror("signal");
      exit(1);
    }
  if((int) signal(SIGTERM,sigcatch) < 0)
    {
      perror("signal");
      exit(1);
    }

  if( argc == 2 ) {
    tty_devname = argv[1];
  } else {
    tty_devname = TTYDEV;
  }

  /////////////// acquire lock on TTY and also set its parameters
  memset(&tio,0,sizeof(tio));
  tio.c_iflag=0;
  tio.c_oflag=0;
  tio.c_cflag=CS8|CREAD|CLOCAL;           // 8n1, see termios.h for more information
  tio.c_lflag=0;
  tio.c_cc[VMIN]=1;
  tio.c_cc[VTIME]=5;
 
  //  tty_fd=open(TTYDEV, O_RDWR | O_NONBLOCK);
  tty_fd=open(tty_devname, O_RDWR );
  if( tty_fd == -1 ) {
    fprintf( stderr, "Can't open device %s\n", tty_devname );
    return -1;
  }
  if(flock( tty_fd, LOCK_EX | LOCK_NB ) == -1) {
    fprintf( stderr, "flock() locking failed with %s (%d)\n", strerror(errno), errno );
    if( errno == EWOULDBLOCK ) {
      fprintf( stderr, "Another using is using the TTY. Try overriding their access.\n" );
    }
    close(tty_fd);
    return 1;
  }

  cfsetospeed(&tio,B115200);            // 115200 baud
  cfsetispeed(&tio,B115200);            // 115200 baud
 
  tcsetattr(tty_fd,TCSANOW,&tio);

  /////////// stdin/stdout set to "raw" mode with canonical (non-blocking)
  memset(&new_stdout,0,sizeof(new_stdout));
  memset(&old_stdout,0,sizeof(old_stdout));
  if( tcgetattr(STDOUT_FILENO, &old_stdout) < 0 )
    return (-1);

  memset(&new_stdin,0,sizeof(new_stdin));
  memset(&old_stdin,0,sizeof(old_stdin));
  if( tcgetattr(STDIN_FILENO, &old_stdin) < 0 )
    return (-1);
  
  new_stdout = old_stdout;
  new_stdin = old_stdin;

  new_stdin.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  new_stdin.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  new_stdin.c_cflag &= ~(CSIZE | PARENB);
  new_stdin.c_cflag |= CS8;
  new_stdin.c_oflag &= ~(OPOST);
  new_stdin.c_cc[VMIN] = 1;
  new_stdin.c_cc[VTIME] = 0;
  if(tcsetattr(STDIN_FILENO, TCSAFLUSH, &new_stdin))
    return -1;

  //  printf("Please start with %s /dev/ttyS1 (for example)\n",argv[0]);
  new_stdout.c_iflag=0;
  new_stdout.c_oflag=0;
  new_stdout.c_cflag=0;
  new_stdout.c_lflag=0;
  new_stdout.c_cc[VMIN]=1;
  new_stdout.c_cc[VTIME]=0;
  tcsetattr(STDOUT_FILENO,TCSANOW,&new_stdout);
  tcsetattr(STDOUT_FILENO,TCSAFLUSH,&new_stdout);
//  fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);       // make the reads non-blocking


  // jail myself in case someone finds a way to break me
  // (note: jail AFTER the file handle to the console is opened ;)
  mkdir("/tmp/jail");
  chroot("/tmp/jail");  // need to be setuid to run chroot
  setuid(33); // set my ID to www-data once this is called so I can kill myself

  // just do a quick print to stderr to validate the jail everytime i run...
  //m = sprintf(dirstr, "Checking jail...");
  //write(STDERR_FILENO, dirstr, m);
  n = scandir("/", &namelist, 0, alphasort);
  if (n < 0) {
    // no need to mention anything if this is the case...
    //perror("scandir");
    //m = sprintf(dirstr, "\n\rlooks good, carry on.\n\r");
    //write(STDERR_FILENO, dirstr, m);
  }
  else {
    m = sprintf(dirstr, "\n\rchroot jail is broken! I can see the contents of /...this is bad.\n\r");
    write(STDERR_FILENO, dirstr, m);
    while (n--) {
      m = sprintf(dirstr, "%s\n\r", namelist[n]->d_name);
      write(STDERR_FILENO, dirstr, m);
      free(namelist[n]);
    }
    free(namelist);
  }

  while (c != 0x04) // EOF check
    {
      // watch stdin
      FD_ZERO(&serialfds);
      FD_SET(0, &serialfds);
      FD_SET(tty_fd, &serialfds);

      tv.tv_sec = 0;
      tv.tv_usec = 100;
      //      retval = select(1, &serialfds, NULL, NULL, &tv);
      retval = select(tty_fd +  1, &serialfds, NULL, NULL, NULL);

#if 0
      // relay chars from the serial board
      if (read(tty_fd,&c,1)>0)        write(STDOUT_FILENO,&c,1);

      // tty always passes through
      if( FD_ISSET(0, &serialfds) ) {
	if (read(0,&c,1)>0)  write(tty_fd,&c,1);
      }
#endif

#if 1
      if( retval ) {
	if( FD_ISSET(tty_fd, &serialfds)) {
	  count = read(tty_fd, bfr, sizeof(bfr));
	  if( count )
	    write(STDOUT_FILENO, bfr, count);
	} 
	if( FD_ISSET(0, &serialfds) ) {
	  count = read(0, bfr, sizeof(bfr));
	  if( count )
	    write(tty_fd, bfr, count);
	}
      }
#endif

    }
 
  if( flock(tty_fd, LOCK_UN) == -1 ) {
    fprintf( stderr,  "flock() unlock failed with %s (%i)\n", strerror(errno), errno );
  }
  close(tty_fd);

  tcsetattr(STDOUT_FILENO, TCSAFLUSH, &old_stdout);
  tcsetattr(STDIN_FILENO, TCSAFLUSH, &old_stdin);

  return 0;
}

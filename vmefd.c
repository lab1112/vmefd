//==============================================================================
// File        : vmefd.c
// Project     : Remote control extended 
// Revision    : $Revision$
// Author      : $Author$
// Date        : $Date$ 
// Description : fd based vme driver   
//==============================================================================
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <sys/psinfo.h>
#include <sys/osinfo.h>
#include <sys/sys_msg.h>
#include <sys/kernel.h>
#include <sys/sendmx.h>
#include <sys/prfx.h>
#include <sys/sched.h>
#include <sys/io_msg.h>
#include <sys/fd.h>
#include <time.h>
//******************************************************************************
// GLOBAL VARS
//******************************************************************************
//******************************************************************************
// LOCAL DEFINE
//******************************************************************************
#define  PFLAGS  (_PPF_SERVER|_PPF_PRIORITY_REC)

#define A16D08  0
#define A16D16  1
#define A24D08  2
#define A24D16  3

//#define _DEBUG
//******************************************************************************
// LOCAL TYPEDEF
//******************************************************************************
struct ocb {
  mode_t mode;
  int    count;
  off_t  offset;
  short int unit;
};
//******************************************************************************
// LOCAL VARS
//******************************************************************************
struct _sysmsg_version_reply version_stamp = {"Vme.fd",__DATE__,410,'X',0} ;
union {
  msg_t				type;
  msg_t				status;
  struct _sysmsg_hdr		sysmsg;
  struct _sysmsg_hdr_reply	version_reply;
  struct _io_open		open;
  struct _io_open_reply		open_reply;
  struct _io_write		write;
  struct _io_write_reply	write_reply;
  struct _io_read		read;
  struct _io_read_reply		read_reply;
  struct _io_fstat		fstat;
  struct _io_fstat_reply	fstat_reply;
  struct _io_utime		utime;
  struct _io_utime_reply	utime_reply;
  struct _io_chmod		chmod;
  struct _io_chmod_reply	chmod_reply;
  struct _io_chown		chown;
  struct _io_chown_reply	chown_reply;
  struct _io_lseek		lseek;
  struct _io_lseek_reply	lseek_reply;
  struct _io_close		close;
  struct _io_close_reply	close_reply;
  struct _io_dup		dup;
  struct _io_dup_reply		dup_reply;
} msg;
struct _io_fstat_reply fstat_reply;
struct _osinfo osdata;
struct _psinfo psdata;
struct _psinfo3 psdata3;
struct _fd_ctrl	*fd_ctrl;
time_t __far *timep;

mode_t perm_mode[4] = { S_IREAD, S_IWRITE, S_IREAD|S_IWRITE };

unsigned char dump[0x20000];//dump[32768] ; // tmp read/write buf to byte access
//******************************************************************************
// LOCAL PROTO
//******************************************************************************
//******************************************************************************
// LOCAL MACRO
//******************************************************************************
//******************************************************************************
// FUNCTIONS
//******************************************************************************
//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
void main(int argc, char *argv[]) {
  struct _mxfer_entry mx_entry[2];
  int devno;
  off_t endmem;
  pid_t pid;
  msg_t type;
  int nbytes;
  mode_t mode;
  struct ocb *ocb;
  off_t offset;
  unsigned sel, selA, selB;
  int size;
  
  #ifdef _DEBUG
    int fd;
    char test = 0xAA;
    int counter=0;
  #endif

  if(qnx_osinfo(0, &osdata) == -1) {perror("osinfo"); exit(EXIT_FAILURE);}
  timep = MK_FP(osdata.timesel, offsetof(struct _timesel, seconds));

  endmem = (off_t)0x20000 ;

  if(qnx_psinfo(PROC_PID, 0, &psdata, 0, NULL) == -1){ perror("psinfo") ;        exit(EXIT_FAILURE) ; }
  if((devno = qnx_device_attach()) == -1){             perror("device_attach") ; exit(EXIT_FAILURE) ; }

  fstat_reply.status = EOK;
  fstat_reply.stat.st_rdev = (dev_t) ((getnid() << 16L) + (devno << 10L)) + 1;
  fstat_reply.stat.st_mode = S_IFREG|S_IREAD|S_IWRITE;
  fstat_reply.stat.st_size = endmem;
  fstat_reply.stat.st_nlink = 1;
  fstat_reply.stat.st_ftime =
  fstat_reply.stat.st_mtime =
  fstat_reply.stat.st_atime =
  fstat_reply.stat.st_ctime = *timep;

  if(qnx_pflags(PFLAGS, PFLAGS, NULL, NULL)) {     perror("pflags");        exit(EXIT_FAILURE); }
  if(!(fd_ctrl = __init_fd(psdata.pid))) {         perror("__init_fd");     exit(EXIT_FAILURE); }
   
  if(qnx_prefix_attach("/dev/vmebus/A16/D08", NULL, A16D08)) { perror("prefix_attach"); exit(EXIT_FAILURE);}
  if(qnx_prefix_attach("/dev/vmebus/A16/D16", NULL, A16D16)) { perror("prefix_attach"); exit(EXIT_FAILURE);}
  if(qnx_prefix_attach("/dev/vmebus/A24/D08", NULL, A24D08)) { perror("prefix_attach"); exit(EXIT_FAILURE);}
  if(qnx_prefix_attach("/dev/vmebus/A24/D16", NULL, A24D16)) { perror("prefix_attach"); exit(EXIT_FAILURE);}

  qnx_scheduler(0, 0, SCHED_RR, 22, 1); 

  (void)signal(SIGHUP, SIG_IGN);
  if(signal(SIGINT,  SIG_IGN) == SIG_IGN) {
    close(0);
    close(1);
    close(2);
  }
  
  selA = qnx_segment_overlay(0x0A0000,0x20000);
  selB = qnx_segment_overlay(0x0B0000,0x8000);
  
  if((selA == -1) || (selB == -1)) {
    return;
  }
  
  #ifdef _DEBUG
    // используем com-порт для дебага
    fd = creat("//6/dev/ser2", S_IRUSR | S_IWUSR);
    
  #endif 

  for(;;) 
  {
    pid = Receive(0, &msg, sizeof(msg));
    type = msg.type;
    msg.status = ENOSYS;
    nbytes = sizeof(msg.status);

    switch(type) 
    {
      case _SYSMSG:
        switch(msg.sysmsg.subtype) 
        {
          case _SYSMSG_SUBTYPE_VERSION:
            msg.version_reply.status = EOK;
            _setmx(&mx_entry[0], &msg, sizeof(msg.version_reply));
            _setmx(&mx_entry[1], &version_stamp, sizeof(version_stamp));
            Replymx(pid, 2, &mx_entry);
            nbytes = 0;
            break;
         }
      break;

      case _IO_HANDLE:
        switch(mode = msg.open.oflag) {
          case _IO_HNDL_INFO:
            goto open2;
          case _IO_HNDL_CHANGE:
            __get_pid_info(pid, &psdata3, fd_ctrl);
            if(psdata3.euid != 0) {
              msg.status = EPERM;
              break;
            }
            goto open2;
          case _IO_UTIME:
            mode |= S_IWRITE;
            goto open1;
        }
      break;
      case _IO_OPEN:
        mode = perm_mode[msg.open.oflag & O_ACCMODE];
open1:
        __get_pid_info(pid, &psdata3, fd_ctrl);
        mode &= (mode & 077) | /* re-use the lower 6 bits for hande */
        (psdata3.euid == 0) ? S_IRWXU :
        (psdata3.euid == fstat_reply.stat.st_uid) ? (fstat_reply.stat.st_mode & S_IRWXU) :
        (psdata3.egid == fstat_reply.stat.st_gid) ?
          (fstat_reply.stat.st_mode & S_IRWXG) << 3 :
          (fstat_reply.stat.st_mode & S_IRWXO) << 6;
        if((mode & S_IRWXU) == 0) {
          msg.status = EPERM;
          break;
        }
open2:
        if(msg.open.path[0] != '\0') {
          msg.status = ENOENT;
          break;
        }
		if(!(ocb = (struct ocb *)malloc(sizeof(struct ocb*)))) {
		  msg.status = ENOMEM;
		  break;
		}
        memset(ocb, 0, sizeof(struct ocb));
        ocb->mode = mode;
        ocb->count++;
        ocb->unit = msg.open.unit;
        if(qnx_fd_attach(pid, msg.open.fd,0,0,0,0,(unsigned)ocb) == -1) 
        {
          free(ocb);
          msg.status = errno;
          break;
        }
        msg.status = EOK;            
      break;
      case _IO_DUP:
        ocb = (struct ocb *)__get_fd(msg.dup.src_pid ? msg.dup.src_pid : pid,msg.dup.src_fd, fd_ctrl);
        if(ocb == (struct ocb *)0 || ocb == (struct ocb *)-1) {
          msg.status = EBADF;
          break;
        }
        if(qnx_fd_attach(pid, msg.dup.dst_fd,0, 0, 0, 0, (unsigned)ocb) == -1) {
          msg.status = errno;
          break;
        }
        ocb->count++;
        msg.status = EOK;            
      break;

    case _IO_CLOSE:
      ocb = (struct ocb *)__get_fd(pid, msg.close.fd, fd_ctrl);
      if((ocb == (struct ocb *)0) || (ocb == (struct ocb *)-1)) {
        msg.status = EBADF;
        break;
      }
      if(--ocb->count==0){ 
        free(ocb); 
        qnx_fd_attach(pid, msg.close.fd,0,0,0,0,0);
      }
      msg.status = EOK;
      break;

    case _IO_STAT:
      if(msg.open.path[0] != '\0') 
      {
         msg.status = ENOENT;
         break;
      }
      Reply(pid, &fstat_reply, sizeof(fstat_reply));
      nbytes = 0;
      break;

    case _IO_FSTAT:
      ocb = (struct ocb *)__get_fd(pid, msg.fstat.fd, fd_ctrl);
      if(ocb == (struct ocb *)0 || ocb == (struct ocb *)-1) 
      {
        msg.status = EBADF;
        break;
      }
      Reply(pid, &fstat_reply, sizeof(fstat_reply));
      nbytes = 0;
      break;

    case _IO_CHMOD:
      ocb = (struct ocb *)__get_fd(pid, msg.chmod.fd, fd_ctrl);
      if(ocb == (struct ocb *)0 || ocb == (struct ocb *)-1) 
      {
        msg.status = EBADF;
        break;
      }
      if((ocb->mode & 077) != _IO_HNDL_CHANGE) 
      {
        msg.status = EBADF;
        break;
      }
      fstat_reply.stat.st_mode = msg.chmod.mode;
      msg.chmod_reply.status = EOK;
      nbytes = sizeof(msg.chmod_reply);
      break;
      
    case _IO_CHOWN:
      ocb = (struct ocb *)__get_fd(pid, msg.chown.fd, fd_ctrl);
      if(ocb == (struct ocb *)0 || ocb == (struct ocb *)-1) 
      {
        msg.status = EBADF;
        break;
      }
      if((ocb->mode & 077) != _IO_HNDL_CHANGE) 
      {
        msg.status = EBADF;
        break;
      }
      fstat_reply.stat.st_uid = msg.chown.uid;
      fstat_reply.stat.st_gid = msg.chown.gid;
      msg.chown_reply.status = EOK;
      nbytes = sizeof(msg.chown_reply);
      break;

    case _IO_UTIME:
      ocb = (struct ocb *)__get_fd(pid, msg.utime.fd, fd_ctrl);
      if(ocb == (struct ocb *)0 || ocb == (struct ocb *)-1) 
      {
        msg.status = EBADF;
        break;
      }
      if((ocb->mode & S_IWRITE) == 0) 
      {
        msg.status = EBADF;
        break;
      }
      if(msg.utime.cur_flag) 
      {
        fstat_reply.stat.st_mtime =
        fstat_reply.stat.st_atime = *timep;
      } else {
        fstat_reply.stat.st_mtime = msg.utime.modtime;
        fstat_reply.stat.st_atime = msg.utime.actime;
      }
      msg.utime_reply.status = EOK;
      nbytes = sizeof(msg.utime_reply);
      break;

    case _IO_LSEEK:
      ocb = (struct ocb *)__get_fd(pid, msg.lseek.fd, fd_ctrl);
      if(ocb == (struct ocb *)0 || ocb == (struct ocb *)-1) 
      {
        msg.status = EBADF;
        break;
      }
      offset = msg.lseek.offset;
      msg.status = EOK;
      switch(msg.lseek.whence) 
      {
      case SEEK_CUR:
        offset += ocb->offset; if(offset<0) offset = 0;
        /* fall through */
      case SEEK_SET:
        if(offset < 0) {
          msg.status = EINVAL;
        } else {
          if(offset > endmem) offset = endmem;
          ocb->offset = msg.lseek_reply.offset = offset;
        }
        break;
      case SEEK_END:
        if(offset > endmem) {
          offset = 0;
        } else {
          offset = endmem - offset;
        }
        ocb->offset = msg.lseek_reply.offset = offset;
        break;
      }
      nbytes = sizeof(msg.lseek_reply);
      break;

    case _IO_READ:
      ocb = (struct ocb *)__get_fd(pid, msg.read.fd, fd_ctrl);
      if(ocb == (struct ocb *)0 || ocb == (struct ocb *)-1) 
      {
        msg.status = EBADF;
        break;
      }
      if((ocb->mode & S_IREAD) == 0) 
      {
        msg.status = EBADF;
        break;
      }
      
      size = msg.read.nbytes;
      if(ocb->offset + size > endmem)  size = endmem - ocb->offset;
      
      if(((ocb->unit) == A16D08) || ((ocb->unit) == A16D16))
        sel = selB;
      else
        sel = selA;

      msg.read_reply.status = EOK;
      msg.read_reply.nbytes = size;
      msg.read_reply.zero   = 0;

      // Чтение данных в зависимости от типа доступа (1-байтовый или 2-байтовый)
      if(((ocb->unit) == A16D08) || ((ocb->unit) == A24D08)) {
        unsigned int i;
        volatile unsigned char __far* pvme;
        pvme = (volatile unsigned char __far *)MK_FP(sel,ocb->offset);
        for(i=0;i<size;i++) dump[i] = pvme[i];
      }
      else {
        unsigned int i;
        volatile unsigned short __far* pvme;
        unsigned short *sptr = (unsigned short *)dump;
        pvme = (volatile unsigned short __far *)MK_FP(sel,ocb->offset);
        for(i=0;i<(size/2);i++) sptr[i] = pvme[i];
      }
      
      _setmx(&mx_entry[1],dump,size) ;
      _setmx(&mx_entry[0],&msg,sizeof(struct _io_read_reply)-sizeof(msg.read_reply.data));
      Replymx(pid, 2, &mx_entry);
      nbytes = 0;
      ocb->offset += size;
      fstat_reply.stat.st_atime = *timep;
      break;

    case _IO_WRITE:
      ocb = (struct ocb *)__get_fd(pid, msg.write.fd, fd_ctrl);
      if(ocb == (struct ocb *)0 || ocb == (struct ocb *)-1) 
      {
        msg.status = EBADF;
        break;
      }
      if((ocb->mode & S_IWRITE) == 0) 
      {
        msg.status = EBADF;
        break;
      } 
 
      size = msg.write.nbytes;
      
      if(ocb->offset + size > endmem)  size = endmem - ocb->offset;

      if(((ocb->unit) == A16D08) || ((ocb->unit) == A16D16))
        sel = selB;
      else
        sel = selA;

      _setmx(&mx_entry[0],dump,size) ;
      msg.write_reply.nbytes = Readmsgmx(pid,sizeof(struct _io_write)-sizeof(msg.write.data),1, &mx_entry);

      if(((ocb->unit) == A16D08) || ((ocb->unit) == A24D08)) {
        unsigned int i;
        volatile unsigned char __far* pvme;
        pvme = (volatile unsigned char __far *)MK_FP(sel,ocb->offset);
        for(i=0;i<size;i++) pvme[i] = dump[i];
      }
      else {
        unsigned int i;
        volatile unsigned short __far* pvme;
        unsigned short *sptr = (unsigned short *)dump;
        pvme = (volatile unsigned short __far *)MK_FP(sel,ocb->offset);
        for(i=0;i<(size/2);i++) pvme[i] = sptr[i];
      }
	  
      msg.write_reply.status = EOK;
      msg.write_reply.nbytes = nbytes;
      Reply(pid, &msg.write_reply, sizeof(msg.write_reply));
      nbytes = 0;
	  
      ocb->offset += size;
      fstat_reply.stat.st_mtime = fstat_reply.stat.st_atime = *timep;
      break;
    }
    if(nbytes){ Reply(pid, &msg, nbytes); }
  }
  qnx_segment_free(selB);
  qnx_segment_free(selA);
}
//******************************************************************************
// END OF FILE
//******************************************************************************


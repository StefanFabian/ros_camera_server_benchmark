/*
 * LD_PRELOAD shim that masks v4l2loopback quirks tripping usb_cam_node_exe.
 *
 * Primary fix (specified): VIDIOC_G_PARM returns parm.capture.capability=0
 * from v4l2loopback, which makes usb_cam abort with
 * "V4L2_CAP_TIMEPERFRAME not supported". The shim ORs the flag in.
 *
 * Secondary fixes (necessary to satisfy "no SIGABRT" + ~30 Hz publish):
 *
 *  - VIDIOC_REQBUFS with V4L2_MEMORY_MMAP and count=N is silently capped to
 *    v4l2loopback's max_buffers (default 2); usb_cam hardcodes count=4 and
 *    aborts with "Insufficient buffer memory on device" when fewer come
 *    back. The shim records the driver-supplied buffer count, then patches
 *    the returned struct to advertise the originally requested count so the
 *    caller's `count_returned < count_requested` check passes.
 *
 *  - VIDIOC_QUERYBUF/QBUF/DQBUF with index >= max_buffers fails -EINVAL,
 *    and v4l2loopback already %-wraps inside used_buffers internally. The
 *    shim remaps `index` to `index % used_buffers` before the syscall and
 *    restores the original index in the returned struct, so the caller
 *    sees N logical buffers backed by used_buffers physical ones.
 *
 * Indexing uses a small fd table; entries reset on close(). All other
 * ioctls pass through unchanged.
 */
#define _GNU_SOURCE
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <dlfcn.h>
#include <errno.h>
#include <stdarg.h>
#include <unistd.h>

#define SHIM_FD_MAX 1024

typedef int (*real_ioctl_t)(int fd, unsigned long request, ...);
typedef int (*real_close_t)(int fd);

static unsigned int g_used_buffers[SHIM_FD_MAX];

static real_ioctl_t resolve_real_ioctl(void)
{
  static real_ioctl_t real_ioctl = NULL;
  if (!real_ioctl) {
    real_ioctl = (real_ioctl_t) dlsym(RTLD_NEXT, "ioctl");
  }
  return real_ioctl;
}

static real_close_t resolve_real_close(void)
{
  static real_close_t real_close = NULL;
  if (!real_close) {
    real_close = (real_close_t) dlsym(RTLD_NEXT, "close");
  }
  return real_close;
}

/* Compare on the low 32 bits because ioctl request macros expand to a
 * signed int that is sign-extended to unsigned long when passed varargs;
 * the constant on the right of `==` would otherwise differ from the
 * caller-provided value (e.g. 0xffffffffc0cc5615 vs 0xc0cc5615). */
static inline int request_eq(unsigned long request, unsigned long target)
{
  return (request & 0xffffffffu) == (target & 0xffffffffu);
}

int ioctl(int fd, unsigned long request, ...)
{
  void *argp;
  va_list ap;
  va_start(ap, request);
  argp = va_arg(ap, void *);
  va_end(ap);

  real_ioctl_t real_ioctl = resolve_real_ioctl();
  if (!real_ioctl) {
    errno = ENOSYS;
    return -1;
  }

  /* REQBUFS: stash mmap buffer cap, advertise requested count back. */
  if (argp != NULL && request_eq(request, VIDIOC_REQBUFS)) {
    struct v4l2_requestbuffers *req = (struct v4l2_requestbuffers *) argp;
    unsigned int requested = req->count;
    int ret = real_ioctl(fd, request, argp);
    if (ret == 0 && req->memory == V4L2_MEMORY_MMAP &&
        fd >= 0 && fd < SHIM_FD_MAX) {
      g_used_buffers[fd] = req->count;
      if (requested > req->count) {
        req->count = requested;
      }
    }
    return ret;
  }

  /* QUERYBUF/QBUF/DQBUF: remap caller index into driver's range, restore
   * on return so the caller sees the index it asked for. */
  if (argp != NULL &&
      (request_eq(request, VIDIOC_QUERYBUF) ||
       request_eq(request, VIDIOC_QBUF) ||
       request_eq(request, VIDIOC_DQBUF))) {
    struct v4l2_buffer *buf = (struct v4l2_buffer *) argp;
    unsigned int original_index = buf->index;
    unsigned int used = (fd >= 0 && fd < SHIM_FD_MAX) ? g_used_buffers[fd] : 0;
    if (used > 0 && buf->index >= used) {
      buf->index = buf->index % used;
    }
    int ret = real_ioctl(fd, request, argp);
    /* For QUERYBUF/QBUF the caller cares about its own index; for DQBUF
     * the kernel writes the index back. Always present the original. */
    if (request_eq(request, VIDIOC_QUERYBUF) ||
        request_eq(request, VIDIOC_QBUF)) {
      buf->index = original_index;
    }
    return ret;
  }

  int ret = real_ioctl(fd, request, argp);

  if (ret == 0 && argp != NULL && request_eq(request, VIDIOC_G_PARM)) {
    struct v4l2_streamparm *parm = (struct v4l2_streamparm *) argp;
    /* Force TIMEPERFRAME so v4l2loopback (which hardcodes capability=0)
     * plays nice with usb_cam. */
    parm->parm.capture.capability |= V4L2_CAP_TIMEPERFRAME;
    parm->parm.output.capability  |= V4L2_CAP_TIMEPERFRAME;
  }

  return ret;
}

int close(int fd)
{
  if (fd >= 0 && fd < SHIM_FD_MAX) {
    g_used_buffers[fd] = 0;
  }
  real_close_t real_close = resolve_real_close();
  if (!real_close) {
    errno = ENOSYS;
    return -1;
  }
  return real_close(fd);
}

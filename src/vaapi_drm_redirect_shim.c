/*
 * LD_PRELOAD shim that redirects /dev/dri/renderD* opens to the path in
 * BENCH_VAAPI_DRM_DEVICE.
 *
 * Why: ffmpeg_image_transport's encoder calls
 *   av_hwdevice_ctx_create(VAAPI, ..., NULL, NULL, 0)
 * libavutil iterates /dev/dri/renderD%d starting at 128 and stops at the
 * first opener — there is no fallback when vaInitialize() fails. On hosts
 * where /dev/dri/renderD128 is a non-VAAPI card (e.g. NVIDIA dGPU is the
 * primary card, Intel iGPU is renderD129) every VAAPI session aborts
 * before the encoder can be opened. The plugin exposes no parameter for
 * the DRM device, and libva itself has no env var to override the fd.
 *
 * Scope: redirects every open()/openat() of a path matching
 * /dev/dri/renderD* to BENCH_VAAPI_DRM_DEVICE. Only loaded into the
 * republish process during ffmpeg-h264 scenarios when the env var is
 * set; if unset the shim is inert.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>

static const char *redirect_target(const char *path)
{
  if (!path) return NULL;
  if (strncmp(path, "/dev/dri/renderD", 16) != 0) return NULL;
  const char *dst = getenv("BENCH_VAAPI_DRM_DEVICE");
  if (!dst || !*dst) return NULL;
  if (strcmp(path, dst) == 0) return NULL;  /* already pointing at target */
  return dst;
}

typedef int (*real_open_t)(const char *, int, ...);
typedef int (*real_openat_t)(int, const char *, int, ...);

#define HOOK_OPEN(symbol)                                          \
  int symbol(const char *path, int flags, ...)                     \
  {                                                                \
    static real_open_t real = NULL;                                \
    if (!real) real = (real_open_t) dlsym(RTLD_NEXT, #symbol);     \
    mode_t mode = 0;                                               \
    if (flags & (O_CREAT | O_TMPFILE)) {                           \
      va_list ap; va_start(ap, flags);                             \
      mode = va_arg(ap, mode_t); va_end(ap);                       \
    }                                                              \
    const char *dst = redirect_target(path);                       \
    return real(dst ? dst : path, flags, mode);                    \
  }

#define HOOK_OPENAT(symbol)                                                \
  int symbol(int dirfd, const char *path, int flags, ...)                  \
  {                                                                        \
    static real_openat_t real = NULL;                                      \
    if (!real) real = (real_openat_t) dlsym(RTLD_NEXT, #symbol);           \
    mode_t mode = 0;                                                       \
    if (flags & (O_CREAT | O_TMPFILE)) {                                   \
      va_list ap; va_start(ap, flags);                                     \
      mode = va_arg(ap, mode_t); va_end(ap);                               \
    }                                                                      \
    const char *dst = redirect_target(path);                               \
    return real(dirfd, dst ? dst : path, flags, mode);                     \
  }

/* libavutil's hwcontext_vaapi calls open64() (LFS variant); other callers
 * may use open()/openat()/openat64(). Hook all four so the redirect catches
 * whichever route the loaded library takes. */
HOOK_OPEN(open)
HOOK_OPEN(open64)
HOOK_OPENAT(openat)
HOOK_OPENAT(openat64)

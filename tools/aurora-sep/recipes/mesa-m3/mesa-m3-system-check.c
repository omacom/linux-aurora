/* SPDX-License-Identifier: MIT */
/* Check system Mesa admission for one M3 render node, without drawing.
 * This helper never selects /opt/mesa-m3 or enables an experimental GPU.
 * The session hook invokes it with an empty environment and a timeout.
 */
#define _GNU_SOURCE
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>
#include <dlfcn.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static bool
board_compat_matches(const char *bytes, size_t length, const char *soc)
{
   const char *board = !strcmp(soc, "t8122") ? "apple,j613" :
                       !strcmp(soc, "t6030") ? "apple,j516s" : NULL;
   if (!board || !length || bytes[length - 1] != '\0')
      return false;
   unsigned boards = 0, socs = 0;
   size_t offset = 0;
   while (offset < length) {
      size_t remaining = length - offset;
      size_t size = strnlen(bytes + offset, remaining);
      if (!size || size == remaining)
         return false;
      const char *value = bytes + offset;
      if (!strncmp(value, "apple,j", 7)) {
         if (strcmp(value, board))
            return false;
         boards++;
      } else if (!strncmp(value, "apple,t", 7)) {
         if (strncmp(value + 6, soc, strlen(soc) + 1))
            return false;
         socs++;
      }
      offset += size + 1;
   }
   return boards == 1 && socs == 1;
}

static bool
qualified_board(const char *soc)
{
   FILE *file = fopen("/proc/device-tree/compatible", "rb");
   if (!file)
      return false;
   char bytes[4096];
   size_t length = fread(bytes, 1, sizeof(bytes), file);
   bool complete = !ferror(file) && length < sizeof(bytes);
   fclose(file);
   return complete && board_compat_matches(bytes, length, soc);
}

static bool
symbol(void *library, const char *name, void *destination, size_t size)
{
   void *address = dlsym(library, name);
   if (!address || size != sizeof(address))
      return false;
   memcpy(destination, &address, size);
   return true;
}

/* Check what actually loaded, rather than trusting a version or a package name.
 * A role supplied through a legacy prefix, or a deleted implementation, fails.
 */
static bool
system_implementation(void)
{
   FILE *maps = fopen("/proc/self/maps", "r");
   if (!maps)
      return false;
   char line[8192];
   bool egl = false, gallium = false, invalid = false;
   while (fgets(line, sizeof(line), maps)) {
      char *path = strchr(line, '/');
      if (!path)
         continue;
      bool is_egl = strstr(path, "libEGL_mesa.so") != NULL;
      bool is_gallium = strstr(path, "libgallium-") != NULL;
      bool is_asahi = strstr(path, "/dri/asahi_dri.so") != NULL;
      if (!is_egl && !is_gallium && !is_asahi)
         continue;
      if (strncmp(path, "/usr/lib/", 9) || strstr(path, " (deleted)"))
         invalid = true;
      egl |= is_egl;
      gallium |= is_gallium || is_asahi;
   }
   bool complete = !ferror(maps);
   fclose(maps);
   return complete && !invalid && egl && gallium;
}

int
main(int argc, char **argv)
{
   if (argc != 3) {
      fprintf(stderr, "usage: mesa-m3-system-check /dev/dri/renderD<N> t8122|t6030\n");
      return 2;
   }
   const char *expected = !strcmp(argv[2], "t8122") ? "Apple M3 (G15G C0)" :
                          !strcmp(argv[2], "t6030") ? "Apple M3 Pro (G15S B1)" : NULL;
   if (!expected)
      return 2;
   const char *node_prefix = "/dev/dri/renderD";
   size_t prefix_length = strlen(node_prefix);
   if (strncmp(argv[1], node_prefix, prefix_length) ||
       !argv[1][prefix_length] ||
       strspn(argv[1] + prefix_length, "0123456789") != strlen(argv[1] + prefix_length))
      return 2;
   if (!qualified_board(argv[2]))
      return 1;
   struct stat st;
   if (lstat(argv[1], &st) || !S_ISCHR(st.st_mode))
      return 1;

   /* env -i at exec is also necessary: clearenv cannot undo LD_PRELOAD that
    * the dynamic linker processed before main. Do not inherit opt-in flags.
    */
   if (clearenv() ||
       setenv("__EGL_VENDOR_LIBRARY_FILENAMES",
              "/usr/share/glvnd/egl_vendor.d/50_mesa.json", 1) ||
       setenv("LIBGL_DRIVERS_PATH", "/usr/lib/dri", 1) ||
       setenv("MESA_SHADER_CACHE_DISABLE", "true", 1))
      return 1;
   alarm(8);
   void *egl_library = dlopen("/usr/lib/libEGL.so.1", RTLD_NOW | RTLD_LOCAL);
   void *gles_library = dlopen("/usr/lib/libGLESv2.so.2", RTLD_NOW | RTLD_LOCAL);
   if (!egl_library || !gles_library)
      return 1;

   PFNEGLGETPROCADDRESSPROC get_proc = NULL;
   PFNEGLINITIALIZEPROC initialize = NULL;
   PFNEGLBINDAPIPROC bind_api = NULL;
   PFNEGLCHOOSECONFIGPROC choose_config = NULL;
   PFNEGLCREATECONTEXTPROC create_context = NULL;
   PFNEGLMAKECURRENTPROC make_current = NULL;
   PFNEGLDESTROYCONTEXTPROC destroy_context = NULL;
   PFNEGLTERMINATEPROC terminate = NULL;
   const GLubyte *(*get_string)(GLenum) = NULL;
#define LOAD(library, function, name) \
   if (!symbol(library, name, &function, sizeof(function))) return 1
   LOAD(egl_library, get_proc, "eglGetProcAddress");
   LOAD(egl_library, initialize, "eglInitialize");
   LOAD(egl_library, bind_api, "eglBindAPI");
   LOAD(egl_library, choose_config, "eglChooseConfig");
   LOAD(egl_library, create_context, "eglCreateContext");
   LOAD(egl_library, make_current, "eglMakeCurrent");
   LOAD(egl_library, destroy_context, "eglDestroyContext");
   LOAD(egl_library, terminate, "eglTerminate");
   LOAD(gles_library, get_string, "glGetString");
#undef LOAD

   PFNEGLQUERYDEVICESEXTPROC query_devices = NULL;
   PFNEGLQUERYDEVICESTRINGEXTPROC device_string = NULL;
   PFNEGLGETPLATFORMDISPLAYEXTPROC platform_display = NULL;
#define EXT(function, name) do { \
   __eglMustCastToProperFunctionPointerType address = get_proc(name); \
   if (!address || sizeof(address) != sizeof(function)) return 1; \
   memcpy(&function, &address, sizeof(function)); \
} while (0)
   EXT(query_devices, "eglQueryDevicesEXT");
   EXT(device_string, "eglQueryDeviceStringEXT");
   EXT(platform_display, "eglGetPlatformDisplayEXT");
#undef EXT
   EGLint count = 0;
   EGLDeviceEXT devices[16];
   if (!query_devices(0, NULL, &count) || count < 1 || count > 16 ||
       !query_devices(count, devices, &count) || count < 1 || count > 16)
      return 1;
   EGLDeviceEXT selected = EGL_NO_DEVICE_EXT;
   for (EGLint i = 0; i < count; ++i) {
      const char *path = device_string(devices[i], EGL_DRM_RENDER_NODE_FILE_EXT);
      if (path && !strcmp(path, argv[1])) {
         if (selected != EGL_NO_DEVICE_EXT)
            return 1;
         selected = devices[i];
      }
   }
   if (selected == EGL_NO_DEVICE_EXT)
      return 1;

   EGLDisplay display = platform_display(EGL_PLATFORM_DEVICE_EXT, selected, NULL);
   if (display == EGL_NO_DISPLAY || !initialize(display, NULL, NULL))
      return 1;
   EGLContext context = EGL_NO_CONTEXT;
   bool accepted = false;
   EGLConfig config;
   EGLint matches = 0;
   const EGLint attributes[] = {
      EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT_KHR,
      EGL_SURFACE_TYPE, EGL_PBUFFER_BIT, EGL_NONE,
   };
   const EGLint context_attributes[] = { EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE };
   if (!bind_api(EGL_OPENGL_ES_API) ||
       !choose_config(display, attributes, &config, 1, &matches) || matches != 1)
      goto done;
   context = create_context(display, config, EGL_NO_CONTEXT, context_attributes);
   if (context == EGL_NO_CONTEXT ||
       !make_current(display, EGL_NO_SURFACE, EGL_NO_SURFACE, context))
      goto done;
   const char *renderer = (const char *)get_string(GL_RENDERER);
   accepted = renderer && !strcmp(renderer, expected) &&
              system_implementation();
   if (accepted)
      printf("system-mesa=available renderer=%s\n", renderer);
done:
   make_current(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
   if (context != EGL_NO_CONTEXT)
      destroy_context(display, context);
   terminate(display);
   dlclose(gles_library);
   dlclose(egl_library);
   return accepted ? 0 : 1;
}

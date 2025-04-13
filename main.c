#if defined(DEBUG)
#define API_CHECK 1
#endif

#include <GLES3/gl32.h>
#include <gtk/gtk.h>
#if defined(SPIRV)
#define GL_GLEXT_PROTOTYPES
#include <GL/gl.h>
#include <GL/glext.h>
#include <GL/glx.h>
#include "gen/spirv/static_shaders.h"
#else
#include "gen/glsl/static_shaders.h"
#endif

#ifndef DEBUG
#include "gen/glsl/shader.frag.h"
// Redefine GTK casting macros as direct casts
#undef GTK_GL_AREA
#undef GTK_CONTAINER
#undef GTK_WINDOW
#undef GTK_WIDGET
#define GTK_GL_AREA (GtkGLArea*)
#define GTK_CONTAINER (GtkContainer*)
#define GTK_WINDOW (GtkWindow*)
#define GTK_WIDGET (GtkWidget*)
#endif

#if defined(API_CHECK)
void handle_compile_error(GLuint shader) {
  GLint success = 0;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
  if (!success) {
    char logBuffer[4096];
    GLsizei length;
    glGetShaderInfoLog(shader, sizeof(logBuffer), &length, logBuffer);
    printf("Shader compile error.\n%s\n", logBuffer);
  }
}

void handle_link_error(GLuint program) {
  GLint success = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (!success) {
    char logBuffer[4096];
    GLsizei length;
    glGetProgramInfoLog(program, sizeof(logBuffer), &length, logBuffer);
    printf("Shader link error.\n%s\n", logBuffer);
  }
}
#else
#define handle_compile_error(shader) do {} while(0)
#define handle_link_error(program) do {} while(0)
#endif

#if defined(SPIRV)
GLuint create_spirv_shader(const char *source, size_t source_length, GLenum type) {
  GLuint shader = glCreateShader(type);
  glShaderBinary(1, &shader, GL_SHADER_BINARY_FORMAT_SPIR_V, source, source_length);
  glSpecializeShader(shader, "main", 0, 0, 0);
  handle_compile_error(shader);
  return shader;
}
#else
GLuint create_shader(const char *source, GLenum type) {
  GLuint shader = glCreateShader(type);
  glShaderSource(shader, 1, &source, NULL);
  glCompileShader(shader);
  handle_compile_error(shader);
  return shader;
}
#endif

GLuint vba;
GLuint program;
GLuint fragment_shader;
#ifdef SPIRV
GLuint ubo;
#endif

gboolean render(GtkGLArea *area, GdkGLContext *context) {
  GtkAllocation size;
  gtk_widget_get_allocation(GTK_WIDGET(area), &size);
#ifdef SPIRV
  float shader_data[2] = { size.width, size.height }; // same memory layout as struct { float size[2]; }
  glBindBuffer(GL_UNIFORM_BUFFER, ubo);
  glBufferData(GL_UNIFORM_BUFFER, sizeof(shader_data), &shader_data, GL_DYNAMIC_DRAW);
  glBindBufferBase(GL_UNIFORM_BUFFER, 0, ubo); // maps to binding=0 in shader
#endif
  glUseProgram(program);
  glBindVertexArray(vba);
#ifndef SPIRV
  glUniform2f(0, size.width, size.height);
#endif
  glDrawArrays(GL_POINTS, 0, 1);
  return TRUE;
}

#ifdef DEBUG
void load_shader(GLuint shader, const char * filename, GLenum type) {
  FILE * f = fopen(filename, "r");
  if (!f) {
    printf("Failed to open %s\n", filename);
    exit(1);
  }
  fseek(f, 0, SEEK_END);
  long length = ftell(f);
  fseek(f, 0, SEEK_SET);
  char buffer[length + 1];
  fread(buffer, 1, length, f);
  buffer[length] = '\0';
  fclose(f);
  const char * source = buffer;
#if defined(SPIRV)
  glShaderBinary(1, &shader, GL_SHADER_BINARY_FORMAT_SPIR_V, source, length);
  glSpecializeShader(shader, "main", 0, 0, 0);
#else
  glShaderSource(shader, 1, &source, NULL);
  glCompileShader(shader);
#endif
  handle_compile_error(shader);
}

void load_fragment_shader() {
#if defined(SPIRV)
  system("mkdir -p gen/spirv && unifdef -b -x2 -DDEBUG -UNDEBUG -DSPIRV -o gen/spirv/shader.frag shader.frag && glslangValidator -V -o gen/spirv/frag.spv gen/spirv/shader.frag");
  load_shader(fragment_shader, "gen/spirv/frag.spv", GL_FRAGMENT_SHADER);
#else
  system("mkdir -p gen && unifdef -b -x2 -DDEBUG -UNDEBUG -o gen/shader-frag-debug.glsl shader.frag");
  load_shader(fragment_shader, "gen/shader-frag-debug.glsl", GL_FRAGMENT_SHADER);
#endif
}

void fragment_shader_changed(GFileMonitor * file_monitor, GFile *file, GFile *other_file, GFileMonitorEvent event_type, GtkGLArea * area) {
  if (event_type != G_FILE_MONITOR_EVENT_CHANGES_DONE_HINT) {
    // events seem to come in bursts of three. The done hint should be last.
    return;
  }
  load_fragment_shader();
  glLinkProgram(program);
  handle_link_error(program);
  gtk_gl_area_queue_render(area);
}
#endif

void realize(GtkGLArea *area) {
  gtk_gl_area_make_current(area);
#ifdef API_CHECK
  if (gtk_gl_area_get_error (area) != NULL) {
    printf("gtk_gl_area_get_error");
    exit(1);
  }
#endif

  program = glCreateProgram();
#if defined(SPIRV)
  GLuint vertex_shader = create_spirv_shader(VERT_SPV, sizeof(VERT_SPV), GL_VERTEX_SHADER);
  GLuint geometry_shader = create_spirv_shader(GEOM_SPV, sizeof(GEOM_SPV), GL_GEOMETRY_SHADER);
#else
  GLuint vertex_shader = create_shader(shader_vert, GL_VERTEX_SHADER);
  GLuint geometry_shader = create_shader(shader_geom, GL_GEOMETRY_SHADER);
#endif
#ifdef DEBUG
  fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
  load_fragment_shader();
#else
  fragment_shader = create_shader(shader_frag, GL_FRAGMENT_SHADER);
#endif
  glAttachShader(program, vertex_shader);
  glAttachShader(program, geometry_shader);
  glAttachShader(program, fragment_shader);
  glLinkProgram(program);
  handle_link_error(program);
  glGenVertexArrays(1, &vba);
#ifdef SPIRV
  glGenBuffers(1, &ubo);
#endif

#ifdef DEBUG
  GFile * fragment_shader_file = g_file_new_for_path("shader.frag");
  GFileMonitor * file_monitor = g_file_monitor_file(fragment_shader_file, G_FILE_MONITOR_NONE, NULL, NULL);
  g_signal_connect(file_monitor, "changed", G_CALLBACK(fragment_shader_changed), area);
#endif
}

void key_press(GtkWidget * widget, GdkEventKey * event, GtkGLArea * area) {
  if (event->keyval == GDK_KEY_Escape) {
#if defined(DEBUG)
    gtk_main_quit();
#else
    // sys_exit_group (exit all threads) syscall
    asm("mov $231,%rax; mov $0,%rdi; syscall");
#endif
  }
}

int main() {
  gtk_init(NULL, NULL);
  GtkWidget * window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
  GtkWidget * area = gtk_gl_area_new();
  gtk_gl_area_set_auto_render(GTK_GL_AREA(area), FALSE);

  gtk_container_add(GTK_CONTAINER(window), area);
  gtk_window_fullscreen(GTK_WINDOW(window));

  g_signal_connect (area, "realize", G_CALLBACK (realize), NULL);
  g_signal_connect (area, "render", G_CALLBACK (render), NULL);
  g_signal_connect (window, "key-press-event", G_CALLBACK(key_press), area);

  gtk_widget_show_all(window);

  gtk_main();
  return 0;
}

#ifndef DEBUG
// Custom _start entry point that doesn't need crt1.o or libc.
// Still not sure why assembler is needed instead of a C function call.
void _start() {
  asm("call main");
}
#endif


CC = clang
CFLAGS = -fno-plt -fno-unwind-tables -Wall -Werror -Oz
CFLAGS += $(shell pkg-config --cflags gtk+-3.0)

LIBS = -lGL -lgtk-3 -lgdk-3 -lgobject-2.0
DEBUG_LIBS = -lGL $(shell pkg-config --libs gtk+-3.0)

XZ = xz -c -9e --format=lzma --lzma1=preset=9,lc=0,lp=0,pb=0
SSTRIP = ./ELFkickers/bin/sstrip
SHADER_MINIFIER = shader_minifier.exe
SOULCRUSH = ./soulcrush/target/release/soulcrush

EXE = bin/main-4k
DEBUG_EXE = bin/main-debug
SPIRV_EXE = bin/main-spirv

all: $(EXE)

run: $(EXE)
	./$(EXE)

debug: $(DEBUG_EXE)
	./$(DEBUG_EXE)

spirv: $(SPIRV_EXE)
	./$(SPIRV_EXE)

clean:
	rm -rf bin/ obj/ gen/

$(SSTRIP):
	cd ELFkickers; make

$(SHADER_MINIFIER):
	wget https://github.com/laurentlb/Shader_Minifier/releases/download/1.3.6/shader_minifier.exe

$(SOULCRUSH):
	cd soulcrush; cargo build --release

gen/glsl/shader.frag: shader.frag $(SOULCRUSH)
	@mkdir -p gen/glsl
	unifdef -x2 -DNDEBUG -UDEBUG -USPIRV -o $@.tmp $<
	$(SOULCRUSH) $@.tmp > $@

gen/glsl/shader.frag.h: gen/glsl/shader.frag $(SHADER_MINIFIER)
	TERM=xterm mono $(SHADER_MINIFIER) --preserve-externals $< -o $@

gen/glsl/shader.%: shader.%
	@mkdir -p gen/glsl
	unifdef -x2 -DNDEBUG -UDEBUG -USPIRV -o $@ $<

gen/glsl/static_shaders.h: gen/glsl/shader.geom gen/glsl/shader.vert $(SHADER_MINIFIER)
	@mkdir -p gen/glsl
	TERM=xterm mono $(SHADER_MINIFIER) --preserve-externals $(filter gen/glsl/shader.%,$^) -o $@

gen/spirv/%.spv: shader.%
	@mkdir -p gen/spirv
	unifdef -x2 -DDEBUG -DSPIRV -o gen/spirv/tmp.$< $<
	glslangValidator -V -o $@ gen/spirv/tmp.$<

gen/spirv/static_shaders.h: gen/spirv/geom.spv gen/spirv/vert.spv
	@mkdir -p gen/spirv
	python3 embed.py $^ > $@

obj/%.o: %.c gen/glsl/shader.frag.h gen/glsl/static_shaders.h
	@mkdir -p obj
	$(CC) -c $(CFLAGS) -o $@ $<

obj/%-debug.o: %.c gen/glsl/static_shaders.h
	@mkdir -p obj
	$(CC) -c $(CFLAGS) -DDEBUG -o $@ $<

obj/%-spirv.o: %.c gen/spirv/static_shaders.h
	@mkdir -p obj
	$(CC) -c $(CFLAGS) -DSPIRV -DDEBUG -o $@ $<

bin/%: obj/%.o $(SSTRIP)
	@mkdir -p bin
	#$(CC) -o $@ $^ $(LIBS)
	ld \
		-z norelro \
		-z nodelete \
		-z noseparate-code \
		-O1 \
		--orphan-handling=discard \
		--as-needed \
		--no-demangle \
		--gc-sections \
		--hash-style=gnu \
		--no-eh-frame-hdr \
		--no-ld-generated-unwind-info \
		-m elf_x86_64 \
		-dynamic-linker \
		/lib64/ld-linux-x86-64.so.2 \
		-o $@ \
		$< \
		$(LIBS)
	$(SSTRIP) $@

bin/%-debug: obj/%-debug.o
	@mkdir -p bin
	$(CC) -o $@ $^ $(DEBUG_LIBS)

bin/%-spirv: obj/%-spirv.o
	@mkdir -p bin
	$(CC) -o $@ $^ $(DEBUG_LIBS)

%.xz: %
	cat $^ | $(XZ) > $@

%-4k: uncompress-header %.xz
	cat $^ > $@
	chmod a+x $@
	@stat --printf="$@: %s bytes\n" $@

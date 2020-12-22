# How to add Static Library

There are two methods to include static library, [adding it into arch library](#adding-it-into-arch-library) and [adding it as a new library](#adding-it-as-a-new-library).

## Add it into arch library

Makefile or Make.defs in arch can include static libraries for specific architectures as mentioned below. As a convention, the specified static library
is added to *libarch.a* in the final build.
```
VPATH += <LIB_PATH>
EXTRA_LIBS += <LIB_PATH>/<LIB_NAME>.a
```
The *LIB_PATH* should be a relative path from *os/arch/arm/src*.

For example,
```
VPATH += chip/abc
EXTRA_LIBS += chip/abc/libnew.a
```

This allows adding a static library from within the Makefile located at *os/arch/arm/src* folder. The specified library *libnew.a* will be merged under *libarch.a*.
```
$(OUTBIN_DIR)/tinyara$(EXEEXT): $(HEAD_OBJ) board/libboard$(LIBEXT)
	$(Q) echo "LD: tinyara"
	$(Q) $(LD) --entry=__start $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(TINYARA) $(HEAD_OBJ) $(EXTRA_OBJS) \
		--start-group $(LDLIBS) $(EXTRA_LIBS) $(LIBGCC) --end-group -Map $(TOPDIR)/../build/output/bin/tinyara.map
```

## Add it as a new library

TizenRT also allows including a static library as a separate entity, without merging it to an existing library. The following steps describe how to include a static library:

1. Add the static library in *Libtargets.mk*.
    ```
    $(LIBRARIES_DIR)$(DELIM)<LIB_NAME>$(LIBEXT): <LIB_PATH>$(DELIM)<LIB_NAME>$(LIBEXT)
    	$(Q) install <LIB_PATH>$(DELIM)<LIB_NAME>$(LIBEXT) $(LIBRARIES_DIR)$(DELIM)<LIB_NAME>$(LIBEXT)
    ```

2. Add the static library in *FlatLibs.mk*, *ProtectedLibs.mk* and *KernelLibs.mk*.

    For kernel library,
    ```
    TINYARALIBS += $(LIBRARIES_DIR)$(DELIM)<LIB_NAME>$(LIBEXT)
    ```

    For user library,
    ```
    USERLIBS += $(LIBRARIES_DIR)$(DELIM)<LIB_NAME>$(LIBEXT)
    ```

    In flat build, there is no difference between TINYARALIBS and USERLIBS.  
    But in protected build and kernel build, TizenRT splits kernel space and user space. So, new static library should be included at appropriate space.

The *LIB_PATH* should be a relative path from *os*.

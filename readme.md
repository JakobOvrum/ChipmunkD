# ChipmunkD
[Chipmunk Physics](http://chipmunk-physics.net/) bindings for the D programming language.

Currently incomplete; in particular, most constraints are not available yet. Patches welcome.

## Directory Layout

 * `chipmunk` - *ChipmunkD* source package.
 * `visuald` - [VisualD](http://www.dsource.org/projects/visuald) project files.
 * `lib` - *ChipmunkD* static libraries (when built).

## Usage
Compile all *ChipmunkD* source files into a static library, either manually or using the included project files.
A makefile will be added in the future. When building your application, link to both *Chipmunk* and *ChipmunkD*.
A `chipmunk.dll` and `chipmunk.lib` import library (in OMF format, used by DMD) can be found [here](https://github.com/downloads/JakobOvrum/ChipmunkD/chipmunk-windmd-dev_x86_32.rar) for convenience.

## License
*ChipmunkD* is licensed under the terms of the MIT license (see the [LICENSE.txt](https://github.com/JakobOvrum/chipmunkd/blob/master/LICENSE.txt) file for details).

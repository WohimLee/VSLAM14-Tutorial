&emsp;
# cv::FileStorage

XML/YAML/JSON file storage class that encapsulates all the information necessary for writing or reading data to/from a file. 

>头文件
```c++
#include <opencv2/core/persistence.hpp>
```

## Public Types
```c++
enum Mode {
    READ   = 0,
    WRITE  = 1,
    APPEND = 2,
    MEMORY = 4,
    FORMAT_MASK = (7<<3),
    FORMAT_AUTO = 0,
    FORMAT_XML  = (1<<3),
    FORMAT_YAML = (2<<3),
    FORMAT_JSON = (3<<3),
    BASE64 = 64,
    WRITE_BASE64 = BASE64 | WRITE
}
```
 
```c++
enum State {
    UNDEFINED = 0,
    VALUE_EXPECTED = 1,
    NAME_EXPECTED = 2,
    INSIDE_MAP = 4
}
```

&emsp;
## Public Member Functions
```c++
// The constructors
FileStorage ()
 	
// Opens a file
FileStorage (const String &filename, int flags, const String &encoding=String())

// the destructor. calls release()
virtual ~FileStorage ()

// Finishes writing nested structure (should pair startWriteStruct())
void endWriteStruct ()
 	
// Returns the first element of the top-level mapping.
FileNode getFirstTopLevelNode () const

// Returns the current format
int getFormat () const

// Checks whether the file is opened.
virtual bool isOpened () const

// Opens a file
virtual bool open (const String &filename, int flags, const String &encoding=String())
 	
// Returns the specified element of the top-level mapping.
FileNode operator[] (const String &nodename) const

FileNode operator[] (const char *nodename) const

// Closes the file and releases all the memory buffers.
virtual void 	release ()

// Closes the file and releases all the memory buffers.
virtual String 	releaseAndGetString ()

// Returns the top-level mapping. 
FileNode root (int streamidx=0) const

// Starts to write a nested structure (sequence or a mapping). 
void startWriteStruct (const String &name, int flags, const String &typeName=String())

// Simplified writing API to use with bindings.
void write (const String &name, int val)

 
void write (const String &name, double val)
 
void write (const String &name, const String &val)
 
void write (const String &name, const Mat &val)
 
void write (const String &name, const std::vector< String > &val)

// Writes a comment.
void writeComment (const String &comment, bool append=false)

// Writes multiple numbers.
void writeRaw (const String &fmt, const void *vec, size_t len)
```

&emsp;
## Static Public Member Functions
```c++
// Returns the normalized object name for the specified name of a file. 
static String getDefaultObjectName (const String &filename)
```

&emsp;
## Public Attributes
```c++
std::string elname

Ptr< Impl > p

int state
```
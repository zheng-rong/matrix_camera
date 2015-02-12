//-----------------------------------------------------------------------------
#ifndef TLILibImportsH
#define TLILibImportsH TLILibImportsH
//-----------------------------------------------------------------------------
#include <mvIMPACT_CPP/mvIMPACT_acquire.h>

#ifdef __cplusplus
	extern "C" {
#endif

typedef void* MVTLI_HANDLE;
typedef void* MVTLI_INTERFACE_HANDLE;
typedef void* MVTLI_DEVICE_HANDLE;

enum INFO_DATATYPE
{
	INFO_DATATYPE_UNKNOWN     = 0,                  /// Unknown data type
	INFO_DATATYPE_STRING      = 1,                  /// 0-terminated C string (ASCII encoded).
	INFO_DATATYPE_STRINGLIST  = 2,                  /// Concatenated INFO_DATATYPE_STRING list. End of list is signaled with an additional 0.
	INFO_DATATYPE_INT16       = 3,                  /// Signed 16 bit integer.
	INFO_DATATYPE_UINT16      = 4,                  /// unsigned 16 bit integer
	INFO_DATATYPE_INT32       = 5,                  /// signed 32 bit integer
	INFO_DATATYPE_UINT32      = 6,                  /// unsigned 32 bit integer
	INFO_DATATYPE_INT64       = 7,                  /// signed 64 bit integer
	INFO_DATATYPE_UINT64      = 8,                  /// unsigned 64 bit integer
	INFO_DATATYPE_FLOAT64     = 9,                  /// signed 64 bit floating point number.
	INFO_DATATYPE_PTR         = 10,                 /// Pointer type (void*). Size is platform dependent (32 bit on 32 bit platforms).
	INFO_DATATYPE_BOOL8       = 11,                 /// Boolean value occupying 8 bit. 0 for false and anything for true.
	INFO_DATATYPE_SIZET       = 12,                 /// Platform dependent unsigned integer (32 bit on 32 bit platforms).
	INFO_DATATYPE_BUFFER      = 13,                 /// Like a INFO_DATATYPE_STRING but with arbitrary data and no 0 termination.
	INFO_DATATYPE_CUSTOM_ID   = 1000                /// Starting value for custom IDs. 
};

enum INTERFACE_INFO_CMD
{
	INTERFACE_INFO_ID           = 0,                      /// STRING
	INTERFACE_INFO_DISPLAYNAME  = 1,                      /// STRING
	INTERFACE_INFO_TLTYPE       = 2,                      /// STRING
	INTERFACE_INFO_CUSTOM_ID    = 1000,
	INTERFACE_INFO_MAC_STRING = INTERFACE_INFO_CUSTOM_ID, /// STRING
	INTERFACE_INFO_MAC,                                   /// UINT64
	INTERFACE_INFO_IP_STRING,                             /// STRING
	INTERFACE_INFO_IP,                                    /// UINT32
	INTERFACE_INFO_NETMASK_STRING,                        /// STRING
	INTERFACE_INFO_NETMASK,                               /// UINT32
	INTERFACE_INFO_ADVANCED_DEVICE_DISCOVERY_MODE,        /// UINT32
	INTERFACE_INFO_GATEWAY,                               /// UINT32
	INTERFACE_INFO_NAME,                                  /// STRING
	INTERFACE_INFO_MTU,                                   /// UINT32
	INTERFACE_INFO_LINK_SPEED                             /// UINT32
};

enum DEVICE_INFO_CMD
{
	DEVICE_INFO_ID                                  = 0,                          /// STRING      Unique ID of the device.
	DEVICE_INFO_VENDOR                              = 1,                          /// STRING      Device vendor name.
	DEVICE_INFO_MODEL                               = 2,                          /// STRING      Device model name.
	DEVICE_INFO_TLTYPE                              = 3,                          /// STRING      Transport layer technologies that are supported.
	DEVICE_INFO_DISPLAYNAME                         = 4,                          /// STRING      String containing a display name for the device ( including a unique id )   
	DEVICE_INFO_ACCESS_STATUS                       = 5,                          /// INT32       Gets the access mode the GenTL Producer has on the opened device. (DEVICE_ACCESS_STATUS enumeration value).
	DEVICE_INFO_CUSTOM_ID                           = 1000,                       /// Starting value for GenTL Producer custom IDs.
	DEVICE_INFO_MANUFACTURER_SPECIFIC_INFO          = DEVICE_INFO_CUSTOM_ID + 0,  /// STRING      String containing the manufacturer specific info
	DEVICE_INFO_GVCP_MESSAGE_TIMEOUT                = DEVICE_INFO_CUSTOM_ID + 1,  /// UINT32_T    Timeout of the message channel in ms
	DEVICE_INFO_LOGMSGWRITER_NAME                   = DEVICE_INFO_CUSTOM_ID + 2,  /// STRING      String containing the name of the log message writer
	DEVICE_INFO_VERSION                             = DEVICE_INFO_CUSTOM_ID + 3,  /// STRING      String containing the device version register
	DEVICE_INFO_SUPPORTS_USER_DEFINED_NAME          = DEVICE_INFO_CUSTOM_ID + 4,  /// BOOL8
	DEVICE_INFO_PERSISTENT_IP_STRING                = DEVICE_INFO_CUSTOM_ID + 5,
	DEVICE_INFO_PERSISTENT_NETMASK_STRING           = DEVICE_INFO_CUSTOM_ID + 6,
	DEVICE_INFO_PERSISTENT_DEFAULT_GATEWAY_STRING   = DEVICE_INFO_CUSTOM_ID + 7,
	DEVICE_INFO_CURRENT_NETMASK_STRING              = DEVICE_INFO_CUSTOM_ID + 8,
	DEVICE_INFO_CURRENT_DEFAULT_GATEWAY_STRING      = DEVICE_INFO_CUSTOM_ID + 9,
	DEVICE_INFO_INTERFACE_COUNT                     = DEVICE_INFO_CUSTOM_ID + 10,
	DEVICE_INFO_AUTONEG_OPTIMAL_SCPS_VALUE          = DEVICE_INFO_CUSTOM_ID + 11,
	DEVICE_INFO_IP_STRING                           = DEVICE_INFO_CUSTOM_ID + 12,
	DEVICE_INFO_MAC_STRING                          = DEVICE_INFO_CUSTOM_ID + 13,
	DEVICE_INFO_SERIALNUMBER                        = DEVICE_INFO_CUSTOM_ID + 14,
	DEVICE_INFO_USER_DEFINED_NAME                   = DEVICE_INFO_CUSTOM_ID + 15,
	DEVICE_INFO_MAC                                 = DEVICE_INFO_CUSTOM_ID + 16,
	DEVICE_INFO_IP                                  = DEVICE_INFO_CUSTOM_ID + 17,
	DEVICE_INFO_SUBNET                              = DEVICE_INFO_CUSTOM_ID + 18,
	DEVICE_INFO_GATEWAY                             = DEVICE_INFO_CUSTOM_ID + 19,
	DEVICE_INFO_SUPPORTS_IP_LLA                     = DEVICE_INFO_CUSTOM_ID + 20,
	DEVICE_INFO_SUPPORTS_IP_DHCP                    = DEVICE_INFO_CUSTOM_ID + 21,
	DEVICE_INFO_SUPPORTS_IP_PERSISTENT              = DEVICE_INFO_CUSTOM_ID + 22,
	DEVICE_INFO_CURRENT_IP_LLA                      = DEVICE_INFO_CUSTOM_ID + 23,
	DEVICE_INFO_CURRENT_IP_DHCP                     = DEVICE_INFO_CUSTOM_ID + 24,
	DEVICE_INFO_CURRENT_IP_PERSISTENT               = DEVICE_INFO_CUSTOM_ID + 25,
	DEVICE_INFO_TIMESTAMP_TICK_FREQUENCY            = DEVICE_INFO_CUSTOM_ID + 26,
	//DEVICE_INFO_SUPPORTS_MESSAGE_CHANNEL            = DEVICE_INFO_CUSTOM_ID + 27, // no longer used
	DEVICE_INFO_ADAPTERNAME                         = DEVICE_INFO_CUSTOM_ID + 28, /// STRING      String with the GUID of the Interface adapter, this is equal to DEVICE_INFO_CUSTOM_CMD in the used version of the TLI headers
	DEVICE_INFO_PRIMARY_APP_SWITCHOVER_SUPPORTED    = DEVICE_INFO_CUSTOM_ID + 29, /// BOOL8
	DEVICE_INFO_PRIMARY_APP_SWITCHOVER_ENABLE       = DEVICE_INFO_CUSTOM_ID + 30, /// BOOL8
	DEVICE_INFO_PRIMARY_APP_SWITCHOVER_KEY          = DEVICE_INFO_CUSTOM_ID + 31, /// UINT32
};

enum DEVICE_ACCESS_FLAGS
{
	DEVICE_ACCESS_UNKNOWN   = 0,         ///< Not used in a command. Can be used to initialize a variable to query that information.
	DEVICE_ACCESS_NONE      = 1,         ///< This either means that the device is not open because it was not opened before or the access to it was denied.
	DEVICE_ACCESS_READONLY  = 2,         ///< Open the device read only. All Port functions can only read from the device.
	DEVICE_ACCESS_CONTROL   = 3,         ///< Open the device in a way that other hosts/processes can have read only access to the device. Device access level is read/write for this process.
	DEVICE_ACCESS_EXCLUSIVE = 4,         ///< Open the device in a way that only this host/process can have access to the device. Device access level is read/write for this process.
	DEVICE_ACCESS_CUSTOM_ID = 1000,      ///<  Starting value for GenTL Producer custom IDs. 
};


#ifndef GC_CALLTYPE
/* Function declaration modifiers */
#if defined (_WIN32)
#  ifdef _GCTLIDLL
#    define GC_IMPORT_EXPORT __declspec(dllexport)
#  else
#    define GC_IMPORT_EXPORT __declspec(dllimport)
#  endif
#  ifndef _M_X64
#    define GC_CALLTYPE __stdcall
#  else
#    define GC_CALLTYPE /* default */
#  endif
#  ifndef EXTERN_C
#    define EXTERN_C extern "C"
#  endif

#elif defined (__GNUC__) && (__GNUC__ >= 4) && defined (__ELF__)
#  define GC_IMPORT_EXPORT __attribute__((visibility("default")))
#  ifdef __i386__
#    define GC_CALLTYPE __attribute__((stdcall))
#  else
#    define GC_CALLTYPE /* default */
#  endif
#  ifndef EXTERN_C
#    define EXTERN_C extern "C"
#  endif

#else
#  error Unknown platform, file needs adaption
#endif

typedef int GC_ERROR;

#     define GC_API GC_IMPORT_EXPORT GC_ERROR GC_CALLTYPE
#     define GC_API_P(function) typedef GC_ERROR( GC_CALLTYPE *function )
#endif

GC_API_P(PTLOpen)                          ( MVTLI_HANDLE* phTL );
GC_API_P(PTLClose)                         ( MVTLI_HANDLE hTL );
GC_API_P(PTLUpdateInterfaceList)           ( MVTLI_HANDLE hTL, char* bHasChanged, int64_type timeout );
GC_API_P(PTLGetNumInterfaces)              ( MVTLI_HANDLE hTL, unsigned int* cnt );
GC_API_P(PTLGetInterfaceID)                ( MVTLI_HANDLE hTL, unsigned int iIndex, char* psName, size_t* piSize );
GC_API_P(PTLOpenInterface)                 ( MVTLI_HANDLE hTL, const char *sInterfaceName, MVTLI_INTERFACE_HANDLE* phInterface );
GC_API_P(PIFClose)                         ( MVTLI_INTERFACE_HANDLE hInterface );
GC_API_P(PIFGetNumDevices)                 ( MVTLI_INTERFACE_HANDLE hInterface, unsigned int* pNumDevices );
GC_API_P(PIFGetDeviceID)                   ( MVTLI_INTERFACE_HANDLE hInterface, unsigned int index, char * psName, size_t* piSize );
GC_API_P(PIFGetInfo)                       ( MVTLI_INTERFACE_HANDLE hInterface, INTERFACE_INFO_CMD iInfoCmd, INFO_DATATYPE* piType, void* pBuffer, size_t* piSize );
GC_API_P(PTLIMV_IFSetInterfaceParam)       ( MVTLI_INTERFACE_HANDLE hInterface, INTERFACE_INFO_CMD iInfoCmd, INFO_DATATYPE *piType, const void *pBuffer, size_t iSize );
GC_API_P(PIFUpdateDeviceList)              ( MVTLI_INTERFACE_HANDLE hInterface, char* bHasChanged, int64_type timeout );
GC_API_P(PIFGetDeviceInfo)                 ( MVTLI_INTERFACE_HANDLE hInterface, const char* pDevName, DEVICE_INFO_CMD iInfoCmd, INFO_DATATYPE* pType, void* pBuffer, size_t* piSize );
GC_API_P(PIFOpenDevice)                    ( MVTLI_INTERFACE_HANDLE hInterface, const char* pDevName, DEVICE_ACCESS_FLAGS iOpenFlags, MVTLI_DEVICE_HANDLE* pDev );
GC_API_P(PTLIMV_IFGetDeviceInterfaceInfo)  ( MVTLI_INTERFACE_HANDLE hInterface, const char* pDevName, unsigned int interfaceIndex, DEVICE_INFO_CMD iInfoCmd, INFO_DATATYPE *pType, void* pBuffer, size_t* piSize );
GC_API_P(PTLIMV_DevSetInterfaceParam)      ( MVTLI_DEVICE_HANDLE hDev, unsigned int interfaceIndex, DEVICE_INFO_CMD iInfoCmd, const void* pBuffer, size_t bufSize );
GC_API_P(PTLIMV_DevSetParam)               ( MVTLI_DEVICE_HANDLE hDev, DEVICE_INFO_CMD iInfoCmd, const void* pBuffer, size_t bufSize );
GC_API_P(PDevClose)                        ( MVTLI_DEVICE_HANDLE hDev );
GC_API_P(PTLIMV_MACFromSerial)             ( const char* pSerial, char* pBuffer, size_t* pBufSize );
GC_API_P(PTLIMV_IsValidIPv4Address)        ( const char* pData );
GC_API_P(PTLIMV_DoAdressesMatch)           ( const char* pIP1, const char* pNM1, const char* pIP2, const char* pNM2 );
GC_API_P(PTLIMV_ForceIP)                   ( const char* pMACAddress, const char* pNewDeviceIPAddress, const char* pStaticSubnetMask, const char* pStaticDefaultGateway, const char* pAdapterIPAddress, unsigned int timeout_ms );

#ifdef __cplusplus
	}
#endif

#endif // TLILibImportsH

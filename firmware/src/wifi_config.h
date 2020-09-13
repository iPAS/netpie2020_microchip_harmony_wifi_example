/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _WIFI_CONFIG_H    /* Guard against multiple inclusion */
#define _WIFI_CONFIG_H

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

    
 /** Please define the following macros by looking in the NETPIE2020 dashboard.
 */
//#define WIFI_SSID   ""
//#define WIFI_PHRASE ""

#if !defined(WIFI_SSID)
#error "WIFI_SSID does not declared here."
#endif

#if !defined(WIFI_PHRASE)
#error "WIFI_PHRASE does not declared here."
#endif
    

/* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */

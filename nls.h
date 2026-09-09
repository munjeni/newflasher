/* Optional translation support, off unless the build defines ENABLE_NLS.
   Without it _() expands to the string itself, NLS_INIT() to nothing and
   no gettext symbol is referenced, so the default build is unchanged. */

#ifndef NEWFLASHER_NLS_H
#define NEWFLASHER_NLS_H

#ifdef ENABLE_NLS
	#include <locale.h>
	#include <libintl.h>

	#ifndef LOCALEDIR
		#define LOCALEDIR "/usr/share/locale"
	#endif

	#define _(String) gettext(String)

	#define NLS_INIT() do { \
		setlocale(LC_ALL, ""); \
		bindtextdomain("newflasher", LOCALEDIR); \
		bind_textdomain_codeset("newflasher", "UTF-8"); \
		textdomain("newflasher"); \
	} while (0)
#else
	#define _(String) (String)
	#define NLS_INIT() do { } while (0)
#endif

#endif /* NEWFLASHER_NLS_H */

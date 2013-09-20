#ifndef __BIONIC_H__
#define __BIONIC_H__
size_t _strspn(const char *s1, const char *s2);
char * _strpbrk(const char *s1, const char *s2);
char *_strtok(char *str, const char *delim, char **saveptr);
int _strcasecmp(const char *s1, const char *s2);
#endif /* __BIONIC_H__ */

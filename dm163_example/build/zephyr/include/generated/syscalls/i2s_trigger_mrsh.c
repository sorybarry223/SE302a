/* auto-generated by gen_syscalls.py, don't edit */

#include <syscalls/i2s.h>

extern int z_vrfy_i2s_trigger(const struct device * dev, enum i2s_dir dir, enum i2s_trigger_cmd cmd);
uintptr_t z_mrsh_i2s_trigger(uintptr_t arg0, uintptr_t arg1, uintptr_t arg2,
		uintptr_t arg3, uintptr_t arg4, uintptr_t arg5, void *ssf)
{
	_current->syscall_frame = ssf;
	(void) arg3;	/* unused */
	(void) arg4;	/* unused */
	(void) arg5;	/* unused */
	union { uintptr_t x; const struct device * val; } parm0;
	parm0.x = arg0;
	union { uintptr_t x; enum i2s_dir val; } parm1;
	parm1.x = arg1;
	union { uintptr_t x; enum i2s_trigger_cmd val; } parm2;
	parm2.x = arg2;
	int ret = z_vrfy_i2s_trigger(parm0.val, parm1.val, parm2.val);
	_current->syscall_frame = NULL;
	return (uintptr_t) ret;
}


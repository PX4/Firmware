# GDB/Python functions for dealing with NuttX

from __future__ import print_function
import gdb, gdb.types

parse_int = lambda x: int(str(x), 0)

class NX_register_set(object):
	"""Copy of the registers for a given context"""

	v7_regmap = {
		'R13':		0,
		'SP':		0,
		'PRIORITY':	1,
		'R4':		2,
		'R5':		3,
		'R6':		4,
		'R7':		5,
		'R8':		6,
		'R9':		7,
		'R10':		8,
		'R11':		9,
		'EXC_RETURN':	10,
		'R0':		11,
		'R1':		12,
		'R2':		13,
		'R3':		14,
		'R12':		15,
		'R14':		16,
		'LR':		16,
		'R15':		17,
		'PC':		17,
		'XPSR':		18,
	}

	v7em_regmap = {
		'R13':		0,
		'SP':		0,
		'PRIORITY':	1,
		'R4':		2,
		'R5':		3,
		'R6':		4,
		'R7':		5,
		'R8':		6,
		'R9':		7,
		'R10':		8,
		'R11':		9,
		'EXC_RETURN':	10,
		'R0':		27,
		'R1':		28,
		'R2':		29,
		'R3':		30,
		'R12':		31,
		'R14':		32,
		'LR':		32,
		'R15':		33,
		'PC':		33,
		'XPSR':		34,
	}

	regs = dict()

	def __init__(self, xcpt_regs):
		if xcpt_regs is None:
			self.regs['R0']         = self.mon_reg_call('r0')
			self.regs['R1']         = self.mon_reg_call('r1')
			self.regs['R2']         = self.mon_reg_call('r2')
			self.regs['R3']         = self.mon_reg_call('r3')
			self.regs['R4']         = self.mon_reg_call('r4')
			self.regs['R5']         = self.mon_reg_call('r5')
			self.regs['R6']         = self.mon_reg_call('r6')
			self.regs['R7']         = self.mon_reg_call('r7')
			self.regs['R8']         = self.mon_reg_call('r8')
			self.regs['R9']         = self.mon_reg_call('r9')
			self.regs['R10']        = self.mon_reg_call('r10')
			self.regs['R11']        = self.mon_reg_call('r11')
			self.regs['R12']        = self.mon_reg_call('r12')
			self.regs['R13']        = self.mon_reg_call('r13')
			self.regs['SP']         = self.mon_reg_call('sp')
			self.regs['R14']        = self.mon_reg_call('r14')
			self.regs['LR']         = self.mon_reg_call('lr')
			self.regs['R15']        = self.mon_reg_call('r15')
			self.regs['PC']         = self.mon_reg_call('pc')
			#self.regs['XPSR']       = self.mon_reg_call('xPSR')
		else:
			for key in self.v7em_regmap.keys():
				self.regs[key] = int(xcpt_regs[self.v7em_regmap[key]])

	def mon_reg_call(self,register):
		"""
		register is the register as a string e.g. 'pc'
		return integer containing the value of the register
		"""
		str_to_eval = "info registers "+register
		resp = gdb.execute(str_to_eval,to_string = True)
		content = resp.split()[-1]
		try:
			return int(content)
		except:
			return 0

	@classmethod
	def with_xcpt_regs(cls, xcpt_regs):
		return cls(xcpt_regs)

	@classmethod
	def for_current(cls):
		return cls(None)

	def __format__(self, format_spec):
		return format_spec.format(
			registers	 = self.registers
			)

	@property
	def registers(self):
		return self.regs


class NX_task(object):
	"""Reference to a NuttX task and methods for introspecting it"""

	def __init__(self, tcb_ptr):
		self._tcb = tcb_ptr.dereference()
		self._group = self._tcb['group'].dereference()
		self.pid = tcb_ptr['pid']

	@classmethod
	def for_tcb(cls, tcb):
		"""return a task with the given TCB pointer"""
		pidhash_sym = gdb.lookup_global_symbol('g_pidhash')
		pidhash_value = pidhash_sym.value()
		pidhash_type = pidhash_sym.type
		for i in range(pidhash_type.range()[0],pidhash_type.range()[1]):
			pidhash_entry = pidhash_value[i]
			if pidhash_entry['tcb'] == tcb:
				return cls(pidhash_entry['tcb'])
		return None

	@classmethod
	def for_pid(cls, pid):
		"""return a task for the given PID"""
		pidhash_sym = gdb.lookup_global_symbol('g_pidhash')
		pidhash_value = pidhash_sym.value()
		pidhash_type = pidhash_sym.type
		for i in range(pidhash_type.range()[0],pidhash_type.range()[1]):
			pidhash_entry = pidhash_value[i]
			if pidhash_entry['pid'] == pid:
				return cls(pidhash_entry['tcb'])
		return None

	@staticmethod
	def pids():
		"""return a list of all PIDs"""
		pidhash_sym = gdb.lookup_global_symbol('g_pidhash')
		pidhash_value = pidhash_sym.value()
		pidhash_type = pidhash_sym.type
		result = []
		for i in range(pidhash_type.range()[0],pidhash_type.range()[1]):
			entry = pidhash_value[i]
			pid = parse_int(entry['pid'])
			if pid is not -1:
				result.append(pid)
		return result

	@staticmethod
	def tasks():
		"""return a list of all tasks"""
		tasks = []
		for pid in NX_task.pids():
			tasks.append(NX_task.for_pid(pid))
		return tasks

	def _state_is(self, state):
		"""tests the current state of the task against the passed-in state name"""
		statenames = gdb.types.make_enum_dict(gdb.lookup_type('enum tstate_e'))
		if self._tcb['task_state'] == statenames[state]:
			return True
		return False

	@property
	def stack_used(self):
		"""calculate the stack used by the thread"""
		if 'stack_used' not in self.__dict__:
			stack_base = self._tcb['stack_alloc_ptr'].cast(gdb.lookup_type('unsigned char').pointer())
			if stack_base == 0:
				self.__dict__['stack_used'] = 0
			else:
				stack_limit = self._tcb['adj_stack_size']
				for offset in range(0, parse_int(stack_limit)):
					if stack_base[offset] != 0xff:
						break
				self.__dict__['stack_used'] = stack_limit - offset
		return self.__dict__['stack_used']

	@property
	def name(self):
		"""return the task's name"""
		return self._tcb['name'].string()

	@property
	def state(self):
		"""return the name of the task's current state"""
		statenames = gdb.types.make_enum_dict(gdb.lookup_type('enum tstate_e'))
		for name,value in statenames.items():
			if value == self._tcb['task_state']:
				return name
		return 'UNKNOWN'

	@property
	def waiting_for(self):
		"""return a description of what the task is waiting for, if it is waiting"""
		if self._state_is('TSTATE_WAIT_SEM'):
			try:
				waitsem = self._tcb['waitsem'].dereference()
				waitsem_holder = waitsem['holder']
				holder = NX_task.for_tcb(waitsem_holder['htcb'])
				if holder is not None:
					return '{}({})'.format(waitsem.address, holder.name)
				else:
					return '{}(<bad holder>)'.format(waitsem.address)
			except:
				return 'EXCEPTION'
		if self._state_is('TSTATE_WAIT_SIG'):
			return 'signal'
		return ""

	@property
	def is_waiting(self):
		"""tests whether the task is waiting for something"""
		if self._state_is('TSTATE_WAIT_SEM') or self._state_is('TSTATE_WAIT_SIG'):
			return True

	@property
	def is_runnable(self):
		"""tests whether the task is runnable"""
		if (self._state_is('TSTATE_TASK_PENDING') or
			self._state_is('TSTATE_TASK_READYTORUN') or
			self._state_is('TSTATE_TASK_RUNNING')):
			return True
		return False

	@property
	def file_descriptors(self):
		"""return a dictionary of file descriptors and inode pointers"""
		filelist = self._group['tg_filelist']
		filearray = filelist['fl_files']
		result = dict()
		for i in range(filearray.type.range()[0],filearray.type.range()[1]):
			inode = parse_int(filearray[i]['f_inode'])
			if inode != 0:
				result[i] = inode
		return result

	@property
	def registers(self):
		if 'registers' not in self.__dict__:
			registers = dict()
			if self._state_is('TSTATE_TASK_RUNNING'):
				registers = NX_register_set.for_current().registers
			else:
				context = self._tcb['xcp']
				regs = context['regs']
				registers = NX_register_set.with_xcpt_regs(regs).registers

			self.__dict__['registers'] = registers
		return self.__dict__['registers']

	def __repr__(self):
		return "<NX_task {}>".format(self.pid)

	def __str__(self):
		return "{}:{}".format(self.pid, self.name)

	def showoff(self):
		print("-------")
		print(self.pid,end = ", ")
		print(self.name,end = ", ")
		print(self.state,end = ", ")
		print(self.waiting_for,end = ", ")
		print(self.stack_used,end = ", ")
		print(self._tcb['adj_stack_size'],end = ", ")
		print(self.file_descriptors)
		print(self.registers)

	def __format__(self, format_spec):
		return format_spec.format(
			pid              = self.pid,
			name             = self.name,
			state            = self.state,
			waiting_for      = self.waiting_for,
			stack_used       = self.stack_used,
			stack_limit      = self._tcb['adj_stack_size'],
			file_descriptors = self.file_descriptors,
			registers	 = self.registers
			)

class NX_show_task (gdb.Command):
	"""(NuttX) prints information about a task"""

	def __init__(self):
		super(NX_show_task, self).__init__("show task", gdb.COMMAND_USER)

	def invoke(self, arg, from_tty):
		t = NX_task.for_pid(parse_int(arg))
		if t is not None:
			my_fmt = 'PID:{pid}  name:{name}  state:{state}\n'
			my_fmt += '  stack used {stack_used} of {stack_limit}\n'
			if t.is_waiting:
				my_fmt += '  waiting for {waiting_for}\n'
			my_fmt += '  open files: {file_descriptors}\n'
			my_fmt += '  R0  {registers[R0]:#010x} {registers[R1]:#010x} {registers[R2]:#010x} {registers[R3]:#010x}\n'
			my_fmt += '  R4  {registers[R4]:#010x} {registers[R5]:#010x} {registers[R6]:#010x} {registers[R7]:#010x}\n'
			my_fmt += '  R8  {registers[R8]:#010x} {registers[R9]:#010x} {registers[R10]:#010x} {registers[R11]:#010x}\n'
			my_fmt += '  R12 {registers[PC]:#010x}\n'
			my_fmt += '  SP  {registers[SP]:#010x} LR {registers[LR]:#010x} PC {registers[PC]:#010x} XPSR {registers[XPSR]:#010x}\n'
			print(format(t, my_fmt))

class NX_show_tasks (gdb.Command):
	"""(NuttX) prints a list of tasks"""

	def __init__(self):
		super(NX_show_tasks, self).__init__('show tasks', gdb.COMMAND_USER)

	def invoke(self, args, from_tty):
		tasks = NX_task.tasks()
		print ('Number of tasks: ' + str(len(tasks)))
		for t in tasks:
			#t.showoff()
			print(format(t, 'Task: {pid} {name} {state} {stack_used}/{stack_limit}'))

NX_show_task()
NX_show_tasks()

class NX_show_heap (gdb.Command):
	"""(NuttX) prints the heap"""

	def __init__(self):
		super(NX_show_heap, self).__init__('show heap', gdb.COMMAND_USER)
		struct_mm_allocnode_s = gdb.lookup_type('struct mm_allocnode_s')
		preceding_size = struct_mm_allocnode_s['preceding'].type.sizeof
		if preceding_size == 2:
			self._allocflag = 0x8000
		elif preceding_size == 4:
			self._allocflag = 0x80000000
		else:
			raise gdb.GdbError('invalid mm_allocnode_s.preceding size %u' % preceding_size)
			self._allocnodesize = struct_mm_allocnode_s.sizeof

	def _node_allocated(self, allocnode):
		if allocnode['preceding'] & self._allocflag:
			return True
		return False

	def _node_size(self, allocnode):
		return allocnode['size'] & ~self._allocflag

	def _print_allocations(self, region_start, region_end):
		if region_start >= region_end:
			raise gdb.GdbError('heap region {} corrupt'.format(hex(region_start)))
		nodecount = region_end - region_start
		print ('heap {} - {}'.format(region_start, region_end))
		cursor = 1
		while cursor < nodecount:
			allocnode = region_start[cursor]
			if self._node_allocated(allocnode):
				state = ''
			else:
				state = '(free)'
			print( '  {} {} {}'.format(allocnode.address + self._allocnodesize,
                                                  self._node_size(allocnode), state))
			cursor += self._node_size(allocnode) / self._allocnodesize

	def invoke(self, args, from_tty):
		heap = gdb.lookup_global_symbol('g_mmheap').value()
		nregions = heap['mm_nregions']
		region_starts = heap['mm_heapstart']
		region_ends = heap['mm_heapend']
		print( '{} heap(s)'.format(nregions))
		# walk the heaps
		for i in range(0, nregions):
			self._print_allocations(region_starts[i], region_ends[i])

NX_show_heap()

class NX_show_interrupted_thread (gdb.Command):
	"""(NuttX) prints the register state of an interrupted thread when in interrupt/exception context"""

	def __init__(self):
		super(NX_show_interrupted_thread, self).__init__('show interrupted-thread', gdb.COMMAND_USER)

	def invoke(self, args, from_tty):
		regs = gdb.lookup_global_symbol('current_regs').value()
		if regs == 0:
			raise gdb.GdbError('not in interrupt context')
		else:
			registers = NX_register_set.with_xcpt_regs(regs)
			my_fmt = ''
			my_fmt += '  R0  {registers[R0]:#010x} {registers[R1]:#010x} {registers[R2]:#010x} {registers[R3]:#010x}\n'
			my_fmt += '  R4  {registers[R4]:#010x} {registers[R5]:#010x} {registers[R6]:#010x} {registers[R7]:#010x}\n'
			my_fmt += '  R8  {registers[R8]:#010x} {registers[R9]:#010x} {registers[R10]:#010x} {registers[R11]:#010x}\n'
			my_fmt += '  R12 {registers[PC]:#010x}\n'
			my_fmt += '  SP  {registers[SP]:#010x} LR {registers[LR]:#010x} PC {registers[PC]:#010x} XPSR {registers[XPSR]:#010x}\n'
			print (format(registers, my_fmt))

NX_show_interrupted_thread()

class NX_check_tcb(gdb.Command):
	""" check the tcb of a task from a address """
	def __init__(self):
		super(NX_check_tcb,self).__init__('show tcb', gdb.COMMAND_USER)

	def invoke(self,args,sth):
		tasks = NX_task.tasks()
		print("tcb int: ",int(args))
		print(tasks[int(args)]._tcb)
		a = tasks[int(args)]._tcb['xcp']['regs']
		print("relevant registers:")
		regmap = NX_register_set.v7em_regmap
		for reg in regmap:
			hex_addr= hex(int(a[regmap[reg]]))
			eval_string = 'info line *'+str(hex_addr)
			print(reg,": ",hex_addr,)
NX_check_tcb()

class NX_tcb(object):
	def __init__(self):
		pass

	def is_in(self,arg,list):
		for i in list:
			if arg == i:
				return True
		return False

	def find_tcb_list(self,dq_entry_t):
		tcb_list = []
		tcb_ptr = dq_entry_t.cast(gdb.lookup_type('struct tcb_s').pointer())
		first_tcb = tcb_ptr.dereference()
		tcb_list.append(first_tcb)
		next_tcb = first_tcb['flink'].dereference()
		while not self.is_in(parse_int(next_tcb['pid']),[parse_int(t['pid']) for t in tcb_list]):
			tcb_list.append(next_tcb)
			old_tcb = next_tcb
			next_tcb = old_tcb['flink'].dereference()

		return [t for t in tcb_list if parse_int(t['pid'])<2000]

	def getTCB(self):
		list_of_listsnames = ['g_pendingtasks','g_readytorun','g_waitingforsemaphore','g_waitingforsignal','g_inactivetasks']
		tcb_list = []
		for l in list_of_listsnames:
			li = gdb.lookup_global_symbol(l)
			print(li)
			cursor = li.value()['head']
			tcb_list = tcb_list + self.find_tcb_list(cursor)

class NX_check_stack_order(gdb.Command):
	""" Check the Stack order corresponding to the tasks """

	def __init__(self):
		super(NX_check_stack_order,self).__init__('show check_stack', gdb.COMMAND_USER)

	def is_in(self,arg,list):
		for i in list:
			if arg == i:
				return True
		return False

	def find_tcb_list(self,dq_entry_t):
		tcb_list = []
		tcb_ptr = dq_entry_t.cast(gdb.lookup_type('struct tcb_s').pointer())
		first_tcb = tcb_ptr.dereference()
		tcb_list.append(first_tcb)
		next_tcb = first_tcb['flink'].dereference()
		while not self.is_in(parse_int(next_tcb['pid']),[parse_int(t['pid']) for t in tcb_list]):
			tcb_list.append(next_tcb)
			old_tcb = next_tcb
			next_tcb = old_tcb['flink'].dereference()

		return [t for t in tcb_list if parse_int(t['pid'])<2000]

	def getTCB(self):
		list_of_listsnames = ['g_pendingtasks','g_readytorun','g_waitingforsemaphore','g_waitingforsignal','g_inactivetasks']
		tcb_list = []
		for l in list_of_listsnames:
			li = gdb.lookup_global_symbol(l)
			cursor = li.value()['head']
			tcb_list = tcb_list + self.find_tcb_list(cursor)
		return tcb_list

	def getSPfromTask(self,tcb):
		regmap = NX_register_set.v7em_regmap
		a =tcb['xcp']['regs']
		return 	parse_int(a[regmap['SP']])

	def find_closest(self,list,val):
		tmp_list = [abs(i-val) for i in list]
		tmp_min = min(tmp_list)
		idx = tmp_list.index(tmp_min)
		return idx,list[idx]

	def find_next_stack(self,address,_dict_in):
		add_list = []
		name_list = []
		for key in _dict_in.keys():
			for i in range(3):
				if _dict_in[key][i] < address:
					add_list.append(_dict_in[key][i])
					if i == 2: # the last one is the processes stack pointer
						name_list.append(self.check_name(key)+"_SP")
					else:
						name_list.append(self.check_name(key))

		idx,new_address = self.find_closest(add_list,address)
		return new_address,name_list[idx]

	def check_name(self,name):
		if isinstance(name,(list)):
			name = name[0]
		idx = name.find("\\")
		newname = name[:idx]

		return newname

	def invoke(self,args,sth):
		tcb = self.getTCB()
		stackadresses={}
		for t in tcb:
			p = []
			#print(t.name,t._tcb['stack_alloc_ptr'])
			p.append(parse_int(t['stack_alloc_ptr']))
			p.append(parse_int(t['adj_stack_ptr']))
			p.append(self.getSPfromTask(t))
			stackadresses[str(t['name'])] = p
		address = int("0x30000000",0)
		print("stack address  :  process")
		for i in range(len(stackadresses)*3):
			  address,name = self.find_next_stack(address,stackadresses)
			  print(hex(address),": ",name)

NX_check_stack_order()

class NX_run_debug_util(gdb.Command):
	""" show the registers of a task corresponding to a tcb address"""
	def __init__(self):
		super(NX_run_debug_util,self).__init__('show regs', gdb.COMMAND_USER)

	def printRegisters(self,task):
		regmap = NX_register_set.v7em_regmap
		a =task._tcb['xcp']['regs']
		print("relevant registers in ",task.name,":")
		for reg in regmap:
			hex_addr= hex(int(a[regmap[reg]]))
			eval_string = 'info line *'+str(hex_addr)
			print(reg,": ",hex_addr,)

	def getPCfromTask(self,task):
		regmap = NX_register_set.v7em_regmap
		a =task._tcb['xcp']['regs']
		return 	hex(int(a[regmap['PC']]))

	def invoke(self,args,sth):
		tasks = NX_task.tasks()
		if args == '':
			for t in tasks:
				self.printRegisters(t)
				eval_str = "list *"+str(self.getPCfromTask(t))
				print("this is the location in code where the current threads $pc is:")
				gdb.execute(eval_str)
		else:
			tcb_nr = int(args)
			print("tcb_nr = ",tcb_nr)
			t = tasks[tcb_nr]
			self.printRegisters(t)
			eval_str = "list *"+str(self.getPCfromTask(t))
			print("this is the location in code where the current threads $pc is:")
			gdb.execute(eval_str)

NX_run_debug_util()


class NX_search_tcb(gdb.Command):
	""" shot PID's of all running tasks """

	def __init__(self):
		super(NX_search_tcb,self).__init__('show alltcb', gdb.COMMAND_USER)

	def is_in(self,arg,list):
		for i in list:
			if arg == i:
				return True
		return False

	def find_tcb_list(self,dq_entry_t):
		tcb_list = []
		tcb_ptr = dq_entry_t.cast(gdb.lookup_type('struct tcb_s').pointer())
		first_tcb = tcb_ptr.dereference()
		tcb_list.append(first_tcb)
		next_tcb = first_tcb['flink'].dereference()
		while not self.is_in(parse_int(next_tcb['pid']),[parse_int(t['pid']) for t in tcb_list]):
			tcb_list.append(next_tcb)
			old_tcb = next_tcb
			next_tcb = old_tcb['flink'].dereference()

		return [t for t in tcb_list if parse_int(t['pid'])<2000]

	def invoke(self,args,sth):
		list_of_listsnames = ['g_pendingtasks','g_readytorun','g_waitingforsemaphore','g_waitingforsignal','g_inactivetasks']
		tasks = []
		for l in list_of_listsnames:
			li = gdb.lookup_global_symbol(l)
			cursor = li.value()['head']
			tasks = tasks + self.find_tcb_list(cursor)

		# filter for tasks that are listed twice
		tasks_filt = {}
		for t in tasks:
			pid = parse_int(t['pid'])
			if not pid in tasks_filt.keys():
				tasks_filt[pid] = t['name']
		print('{num_t} Tasks found:'.format(num_t = len(tasks_filt)))
		for pid in tasks_filt.keys():
			print("PID: ",pid," ",tasks_filt[pid])

NX_search_tcb()


class NX_my_bt(gdb.Command):
	""" 'fake' backtrace: backtrace the stack of a process and check every suspicious address for the list
	arg: tcb_address$
	(can easily be found by typing 'showtask').
	"""

	def __init__(self):
		super(NX_my_bt,self).__init__('show mybt', gdb.COMMAND_USER)

	def readmem(self,addr):
		'''
		read memory at addr and return nr
		'''
		str_to_eval = "x/x "+hex(addr)
		resp = gdb.execute(str_to_eval,to_string = True)
		idx = resp.find('\t')
		return int(resp[idx:],16)

	def is_in_bounds(self,val):
		lower_bound = int("08004000",16)
		upper_bound = int("080ae0c0",16)
		#print(lower_bound," ",val," ",upper_bound)
		if val>lower_bound and val<upper_bound:
			return True
		else:
			return False
	def get_tcb_from_address(self,addr):
		addr_value = gdb.Value(addr)
		tcb_ptr = addr_value.cast(gdb.lookup_type('struct tcb_s').pointer())
		return tcb_ptr.dereference()

	def resolve_file_line_func(self,addr,stack_percentage):
		gdb.write(str(round(stack_percentage,2))+":")
		str_to_eval = "info line *"+hex(addr)
		#gdb.execute(str_to_eval)
		res = gdb.execute(str_to_eval,to_string = True)
		# get information from results string:
		words = res.split()
		if words[0] != 'No':
			line = int(words[1])
			block = gdb.block_for_pc(addr)
			func = block.function
			if str(func) == "None":
				func = block.superblock.function
			return words[3].strip('"'), line, func

	def invoke(self,args,sth):
		try:
			addr_dec = parse_int(args)   # Trying to interpret the input as TCB address
		except ValueError:
			for task in NX_task.tasks(): # Interpreting as a task name
				if task.name == args:
					_tcb = task._tcb
					break
		else:
			_tcb = self.get_tcb_from_address(addr_dec)

		print("found task with PID: ",_tcb["pid"])
		up_stack = parse_int(_tcb['adj_stack_ptr'])
		curr_sp = parse_int(_tcb['xcp']['regs'][0]) #curr stack pointer
		other_sp = parse_int(_tcb['xcp']['regs'][8]) # other stack pointer
		stacksize = parse_int(_tcb['adj_stack_size']) # other stack pointer

		print("tasks current SP = ",hex(curr_sp),"stack max ptr is at ",hex(up_stack))

		item = 0
		for sp in range(other_sp if curr_sp == up_stack else curr_sp, up_stack, 4):
			mem = self.readmem(sp)
			#print(hex(sp)," : ",hex(mem))
			if self.is_in_bounds(mem):
				# this is a potential instruction ptr
				stack_percentage = (up_stack-sp)/stacksize
				filename,line,func = self.resolve_file_line_func(mem, stack_percentage)
				print('#%-2d ' % item, '0x%08x in ' % mem, func, ' at ', filename, ':', line, sep='')
				item += 1

NX_my_bt()

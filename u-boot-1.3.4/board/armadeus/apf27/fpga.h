/*
 * (C) Copyright 2002-2008
 * Eric Jarrige <eric.jarrige@armadeus.org>
 * Rich Ireland, Enterasys Networks, rireland@enterasys.com.
 * Keith Outwater, keith_outwater@mvis.com.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 *
 */
extern int APF27_init_fpga (u_char * buffer, size_t size);

extern int fpga_pre_fn (int cookie);
extern int fpga_pgm_fn (int assert_pgm, int flush, int cookie);
extern int fpga_cs_fn (int assert_cs, int flush, int cookie);
extern int fpga_init_fn (int cookie);
extern int fpga_done_fn (int cookie);
extern int fpga_clk_fn (int assert_clk, int flush, int cookie);
extern int fpga_wr_fn (int assert_write, int flush, int cookie);
extern int fpga_rdata_fn ( unsigned char *data, int cookie );
extern int fpga_wdata_fn ( unsigned char data, int cookie );
extern int fpga_abort_fn (int cookie);
extern int fpga_post_fn (int cookie);
extern int fpga_busy_fn (int cookie);

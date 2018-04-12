#!/usr/bin/ruby

require 'optparse'
require 'fileutils'

include FileUtils

px4_dir="px4dir"

sitl_gazebo_dir = "sitl_gazebo"
firmware_dir = "Firmware"
px4_fname="px4"


#script dir
@root_dir = __dir__ + '/'

#options
opts = {
  n: 1,
  r: 10000,
  f: "ekf2",
  gazebo_model: "iris",
  workspace: "workspace",
  gazebo: "gazebo",
  catkin_ws: "workspace/catkin_ws",
  base_port: 15010,
  port_step: 10,
  distance: 2,
  firmware_in: "../../../",
  sitl_gazebo_in: "../"
}

#sitl_gazebo
model_opts_open = "<?xml version=\"1.0\" ?>
<options>\n"
model_opts_close = '</options>'


def xspawn(term_name, cmd, debug)
  term = debug ? "xterm -T #{term_name} -hold -e" : ""
  pid = spawn("#{term} #{cmd}", [:out, :err]=>"/dev/null")
  Process.detach(pid)
end

def create_fcu_files(opts,m_num)
  @rc_file="rcS#{m_num}"

  unless File.exist?(@rc_file)
    mkdir_p "rootfs/fs/microsd"
    mkdir_p "rootfs/eeprom"
    touch "rootfs/eeprom/parameters"

    cp_r @mixers_dir, "./"

    #generate rc file
    rc = @firmware_init_str.sub('param set MAV_TYPE',"param set MAV_SYS_ID #{m_num}\nparam set MAV_TYPE")
    rc.sub!('ROMFS/px4fmu_common/','')
    unless opts[:logging]
      rc.sub!(/sdlog2 start.*\n/,'')
      rc.sub!(/logger start.*\n/,'')
    end
    rc.sub!(/.*OPTICAL_FLOW_RAD.*\n/,'') unless opts[:optical_flow]

    rc.sub!(/simulator start -s.*$/,"simulator start -s -u #{@sim_port}")

    rc.gsub!("-r 4000000","-r #{opts[:r]}")

    rc.gsub!("-u 14556","-u #{@mav_port}")
    rc.sub!("mavlink start -x -u #{@mav_port}","mavlink start -x -u #{@mav_port} -o #{@mav_oport}")

    rc.sub!("-u 14557","-u #{@mav_port2}")
    rc.sub!("-r 250 -s HIGHRES_IMU -u #{@mav_port}", "-r #{opts[:imu_rate]} -s HIGHRES_IMU -u #{@mav_port2}") if opts[:imu_rate]
    rc.sub!("-o 14540","-o #{@mav_oport2}")
    rc.sub!("gpssim start","param set MAV_USEHILGPS 1") if opts[:hil_gps]

    File.open(@rc_file, 'w') { |out| out << rc }
  end
end

def check_expanded_path(file_name, dir = nil, msg = nil)
  p = File.expand_path(file_name, dir)

  unless File.exist?(p)
    puts "#{p} does not exist" + msg ? ", #{msg}" : ""
    exit
  end

  p
end

#options
op = OptionParser.new do |op|
  op.banner = "Usage: #{__FILE__} [options] [world_file]"

  op.on("-n NUM", Integer, "number of instances") { |p| opts[:n] = p }
  op.on("-r RATE", Integer, "px4 data rate") { |p| opts[:r] = p }
  op.on("-f FILTER", "px4 filter") { |p| opts[:f] = p }
  op.on("-i NAME", "px4 model init file") { |p| opts[:i] = p }

  op.on("--gazebo_model NAME", "gazebo model name") { |p| opts[:gazebo_model] = p }
  op.on("--imu_rate IMU_RATE", Integer, "imu rate") { |p| opts[:imu_rate] = p }
  op.on("--hil_gps", "turn on hil_gps mode") { opts[:hil_gps] = true }
  op.on("--plugin_lists PATH", "path to mavros pluginlists.yaml") { |p| opts[:plugin_lists] = p }
  op.on("--logging", "turn on logging") { opts[:logging] = true }
  op.on("--debug", "debug mode") { opts[:debug] = true }
  op.on("--nomavros", "without mavros") { opts[:nomavros] = true }
  op.on("--workspace PATH", "path to workspace") { |p| opts[:workspace] = p }
  op.on("--gazebo PATH", "path to gazebo resources") { |p| opts[:gazebo] = p }
  op.on("--catkin_ws PATH", "path to catkin workspace") { |p| opts[:catkin_ws] = p }
  op.on("--base_port PORT", Integer, "base port") { |p| opts[:base_port] = p }
  op.on("--port_step STEP", Integer, "port step") { |p| opts[:port_step] = p }
  op.on("--distance DISTANCE", Integer, "distance between models") { |p| opts[:distance] = p }
  op.on("--firmware_in PATH", "relative path to #{firmware_dir}") { |p| opts[:firmware_in] = p }
  op.on("--sitl_gazebo_in PATH", "relative path to #{sitl_gazebo_dir}") { |p| opts[:sitl_gazebo_in] = p }
  op.on("--optical_flow", "turn on optical flow") { opts[:optical_flow] = true }

  op.on("--restart", "soft restart") do
    opts[:restart] = true
    puts "restarting ..."
  end

  op.on("-h", "help and show defaults") do
    puts op
    puts "Defaults: #{opts}"
    exit
  end
end
op.parse!

#files and dirs
@firmware_dir = check_expanded_path(opts[:firmware_in] + firmware_dir, @root_dir) + '/'

firmware_init_file = check_expanded_path("posix-configs/SITL/init/#{opts[:f]}/#{opts[:i] || opts[:gazebo_model]}", @firmware_dir, "use valid -f, -i, --gazebo_model options")
@mixers_dir = check_expanded_path("ROMFS/px4fmu_common/mixers", @firmware_dir)
firmware_file = check_expanded_path("build/posix_sitl_default/" + px4_fname, @firmware_dir)

sitl_gazebo_dir = check_expanded_path(opts[:sitl_gazebo_in] + sitl_gazebo_dir, @root_dir)

opts[:world] = ARGV[0] ? check_expanded_path(ARGV[0]) : check_expanded_path("worlds/#{opts[:gazebo_model]}.world", sitl_gazebo_dir, "invalid --gazebo_model option")
opts[:workspace] = File.expand_path(opts[:workspace]) + "/"
opts[:gazebo] = File.expand_path(opts[:gazebo])
opts[:catkin_ws] = File.expand_path(opts[:catkin_ws])
opts[:plugin_lists] = File.expand_path(opts[:plugin_lists]) if opts[:plugin_lists]

px4_dir = opts[:workspace] + px4_dir + "/"

#init
cd @root_dir

if opts[:restart]
  system("./kill_px4.sh")
else
  system("./kill_sitl.sh")
end
sleep 1

#start

unless File.exist?(px4_dir + px4_fname)
  mkdir_p px4_dir
  cp firmware_file, px4_dir
end


world_sdf = File.read(opts[:world])

world_name = world_sdf[/<world name="(.*)">/, 1]
world_fname = "#{world_name}.world"
models_opts_fname = "#{world_name}.xml"

world_sdf.sub!(/\n[^<]*<include>[^<]*<uri>model:\/\/iris[^<]*<\/uri>.*?<\/include>/m, "")

model_incs = ""
model_opts = ""
@firmware_init_str = File.read(firmware_init_file)
opts[:n].times do |i|
  x=i*opts[:distance]
  m_index=i
  m_num=i+1

  @mav_port = opts[:base_port] + m_index*opts[:port_step]
  @mav_port2 = @mav_port + 1

  @mav_oport = @mav_port + 5
  @mav_oport2 = @mav_port + 6

  @sim_port = @mav_port + 9

  @gcs_inport = @mav_port + 2000

  model_name="#{opts[:gazebo_model]}#{m_num}"

  cd(px4_dir) {
    mkdir_p model_name

    cd(model_name) {
      create_fcu_files(opts,m_num)

      #run px4
      xspawn("px4-#{m_num}", "../#{px4_fname} -d #{@rc_file}", opts[:debug])
    }
  }

  #generate model
  n = "<name>#{model_name}</name>"
  model_incs += "    <include>
      <uri>model://#{opts[:gazebo_model]}</uri>
      <pose>#{x} 0 0 0 0 0</pose>
      #{n}
    </include>\n" unless world_sdf.include?(n)

  model_opts += "  <model>
    <name>#{model_name}</name>
    <mavlink_udp_port>#{@sim_port}</mavlink_udp_port>\n"
  model_opts += "  </model>\n"

  cd("mavros") {
    sleep 1

    pl="plugin_lists:=#{opts[:plugin_lists]}" if opts[:plugin_lists]
    launch_opts = "fcu_url:=udp://127.0.0.1:#{@mav_oport2}@127.0.0.1:#{@mav_port2} gcs_inport:=#{@gcs_inport}"

    xspawn("mavros-#{m_num}", "./roslaunch.sh #{opts[:catkin_ws]} num:=#{m_num} #{pl} #{launch_opts}", opts[:debug])

  } unless opts[:restart] or opts[:nomavros]

end

if opts[:restart]
  system("gz world -o")
else
  File.open(opts[:workspace] + models_opts_fname, 'w') do |out|
    out << model_opts_open
    out << model_opts
    out << model_opts_close
  end

  #run gzserver
  File.open(opts[:workspace] + world_fname, 'w') do |out|
    world_sdf.each_line {|l|
      out << l
      if l =~ /.*<world.*\n/
        out << model_incs
      end
    }

  end

  cd(opts[:workspace]) {
    xspawn("gazebo", "#{@root_dir}gazebo.sh #{world_fname} #{opts[:gazebo]} #{sitl_gazebo_dir}", opts[:debug])
  }
end

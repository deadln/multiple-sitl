#!/usr/bin/ruby

require 'optparse'
require 'fileutils'

include FileUtils

def xspawn(term_name, cmd, debug, env = {})
  term = debug ? "xterm -T #{term_name} -hold -e" : ""
  pid = spawn(env, "#{term} #{cmd}", [:out, :err]=>"/dev/null")
  Process.detach(pid)
end

def create_fcu_files(m_num, ports)
  rc_file="rcS#{m_num}"

  #generate rc file
  rc = String.new(@contents[:firmware_init_rc])
  #TD
  rc.sub!(/(SCRIPT_DIR=).*/,"\\1#{File.dirname(@abs[:firmware_init_rc])}\nPX4_SIM_MODEL=#{@opts[:i] || @opts[:gazebo_model]}\nPX4_ESTIMATOR=#{@opts[:f]}")

  unless @opts[:logging]
    #TD
    rc.sub!(/.*rc\.logging.*\n/,'')
  end
  rc.sub!(/.*OPTICAL_FLOW_RAD.*\n/,'') unless @opts[:optical_flow]

  rc.sub!(/(simulator start).*/, "\\1 -#{@opts[:udp_sitl] ? 'u' : 'c' } #{ports[:sim]}")

  rc.gsub!(/(mavlink start .*-r) \S+/,"\\1 #{@opts[:r]}")

  rc.sub!(/(udp_gcs_port_local=).*/, "\\1#{ports[:gcs]}")
  rc.sub!(/(udp_offboard_port_local=).*/, "\\1#{ports[:offb]}")
  rc.sub!(/(udp_offboard_port_remote=).*/, "\\1#{ports[:offb_out]}")

  rc.sub!(/(mavlink start .*udp_gcs_port_local)/, "\\1 -o #{ports[:gcs_out]}")

  #rc.sub!("-r 250 -s HIGHRES_IMU -u #{ports[:gcs]}", "-r #{@opts[:imu_rate]} -s HIGHRES_IMU -u #{ports[:offb]}") if @opts[:imu_rate]
  rc.sub!("gpssim start","param set MAV_USEHILGPS 1") if @opts[:hil_gps]

  File.open(rc_file, 'w') { |out|
    rc.each_line {|l|
      if @opts[:onboard_streams] and l =~ /mavlink start.*-m onboard.*\n/
        l.sub!("onboard","magic")
        out << l

        @contents[:onboard_streams].each_line { |sl|
          type, rate = sl.split
          out << "mavlink stream -r #{rate} -s #{type} -u #{ports[:offb]}\n" if type and rate
        }
      else
        out << l
      end
    }
  }

  rc_file
end

def check_expanded_path(file_name, dir = nil, msg = nil)
  p = File.expand_path(file_name, dir)

  unless File.exist?(p)
    puts "#{p} does not exist" + (msg ? ", #{msg}" : "")
    exit
  end

  p
end

def xml_elements(kv)
  s = ""
  kv.each { |k, v| s<<"<#{k}>#{v}</#{k}>" }

  s
end

def check_gazebo_resource(key)
  unless @abs[key]
    a = []
    for sym in [:gazebo, :sitl_gazebo]
      a << @abs[sym]
      ap = File.expand_path(@rels[key], @abs[sym])
      if File.exist?(ap)
        @abs[key] = ap
        break
      end
    end

    unless @abs[key]
      puts @rels[key] + " does not exist in " + a.join(", ")
      exit
    end
  end
end

def expand_and_check()
  #do not check
  for sym in [:workspace, :gazebo, :catkin_ws]
    @abs[sym] = File.expand_path(@opts[sym])
  end

  #check
  for sym in [:firmware, :sitl_gazebo, :plugin_lists, :gazebo_world, :onboard_streams]
    @abs[sym] = check_expanded_path(@opts[sym]) if @opts[sym]
  end

  #set defaults if not set
  for sym in [:firmware, :sitl_gazebo]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:home]) unless @abs[sym]
  end

  #check firmware paths
  for sym in [:firmware_bin, :firmware_etc]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:firmware])
  end
  @abs[:firmware_init_rc] = check_expanded_path(@rels[:firmware_init_rc], @abs[:firmware_etc])

  #gazebo resources
  for sym in [:gazebo_world, :gazebo_model]
    check_gazebo_resource(sym)
  end

  #workspace
  for sym in [:workspace_firmware, :workspace_world]
    @abs[sym] = File.expand_path(@rels[sym], @abs[:workspace])
  end
  @abs[:workspace_model] = File.expand_path(@rels[:gazebo_model], @abs[:workspace])

end

def init_contents()
  #contents
  @contents = {}
  for sym in [:firmware_init_rc, :gazebo_world, :gazebo_model, :onboard_streams]
    @contents[sym] = File.read(@abs[sym]) if @abs[sym]
  end
end

def ports_by_index(i)
  ports = {}
  p = @opts[:ports_base] + i*@opts[:ports_step]
  
  for n in @port_names
    ports[n] = p + @opts["pd_#{n}".to_sym]
  end

  ports
end


def start_mavros()
  @opts[:n].times { |m_index|
    m_num = m_index+1
    ports = ports_by_index(m_index)
    
    #mavros instance
    cd("mavros") {
      pl="plugin_lists:=#{@abs[:plugin_lists]}" if @abs[:plugin_lists]
      launch_opts = "num:=#{m_num} fcu_url:=udp://127.0.0.1:#{ports[:offb_out]}@127.0.0.1:#{ports[:offb]} gcs_inport:=#{ports[:gcs_mavros]} #{pl}"
      launch_suffix = @opts[:single] ? "single" : "num"

      xspawn("mavros-#{m_num}", "./roslaunch.sh #{@abs[:catkin_ws]} px4_#{launch_suffix}.launch #{launch_opts}", @opts[:debug])
      if m_num == 1
        puts 'sleep'
        sleep 3
      end
    }
  } unless @opts[:restart] or @opts[:nomavros]
end

def start_firmware()
  fw_name = @rels[:firmware_bin].split('/').last
  for sym in [:workspace_firmware]
    mkdir_p @abs[sym]
  end

  @opts[:n].times do |m_index|
    m_num = m_index+1
    model_name="#{@opts[:gazebo_model]}#{m_num}"
    ports = ports_by_index(m_index)
    
    #firmware instance
    cd(@abs[:workspace_firmware]) {
      mkdir_p model_name

      cd(model_name) {
        rc_file = create_fcu_files(m_num, ports)
        rc_file="etc/init.d-posix/rcS"

        #run
        xspawn("#{fw_name}-#{m_num}", "#{@abs[:firmware_bin]} #{@opts[:debug] ? '' : '-d'} -i #{m_index} -s #{rc_file} #{@abs[:firmware_etc]}", @opts[:debug], {'PATH'=> ENV['PATH'] + ':../'})
      }
    } unless @opts[:hitl]
  end

end

def start_gazebo()
  #paths
  d = File.dirname(@abs[:workspace_model])
  mkdir_p d
  rm_r d, force: true
  cp_r File.dirname(@abs[:gazebo_model]), d

  #prepare
  model_incs = ""
  model_opts = ""
  port_param = @opts[:udp_sitl] ? 'mavlink_udp_port' : 'mavlink_tcp_port'
  
  @opts[:n].times do |m_index|
    model_name = "#{@opts[:gazebo_model]}#{m_index+1}"
    ports = ports_by_index(m_index)
    
    #generate sdf parts
    n = "<name>#{model_name}</name>"
    x = m_index*@opts[:distance]
    model_incs += "    <include>#{n}<uri>model://#{@opts[:gazebo_model]}</uri><pose>#{x} 0 0 0 0 0</pose></include>\n" unless @contents[:gazebo_world].include?(n)
    model_opts += "      <#{port_param} model=\"#{model_name}\">#{@opts[:hitl] ? ports[:offb] : ports[:sim]}</#{port_param}>\n"
  end

  #gazebo
  if @opts[:restart]
    system("gz world -o")
  else
    #generate model
    File.open(@abs[:workspace_model], 'w') do |out|
      @contents[:gazebo_model].each_line {|l|
        if l =~ /.*<#{port_param}>.*\n/
          out << model_opts
        else
          out << l
        end
      }
    end

    #generate world
    File.open(@abs[:workspace_world], 'w') do |out|
      @contents[:gazebo_world].sub!(/\n[^<]*<include>[^<]*<uri>model:\/\/#{@opts[:gazebo_model]}[^<]*<\/uri>.*?<\/include>/m, "")
      @contents[:gazebo_world].each_line {|l|
        out << l
        if l =~ /.*<world.*\n/
          out << model_incs
        end
      }
    end

    #start gazebo
    cd(@abs[:workspace]) {
      xspawn("gazebo", @abs[:home] + "/gazebo.sh #{@rels[:workspace_world]} #{@abs[:workspace]} #{@abs[:gazebo]} #{@abs[:sitl_gazebo]}", @opts[:debug])
    }
  end

end

####################################

#default options
@opts = {
  n: 1,
  r: 10000,
  f: "ekf2",
  go: {},
  gazebo_model: "iris",

  ports_base: 15010,
  ports_step: 10,
  pd_gcs: 0,
  pd_gcs_out: 5,
  pd_offb: 1,
  pd_offb_out: 6,
  pd_sim: 9,
  pd_gcs_mavros: 2000,
  
  distance: 2,
  build_label: "default",

  workspace: "workspace",
  gazebo: "gazebo",
  catkin_ws: "workspace/catkin_ws",
}

@port_names = []

#parse options
OptionParser.new do |op|
  op.banner = "Usage: #{__FILE__} [options] [world_file]"

  op.on("-n NUM", Integer, "number of instances")
  op.on("-r RATE", Integer, "px4 data rate")
  op.on("-f FILTER", "px4 filter")
  op.on("-i NAME", "px4 model init file")
  op.on("-g PARAM=VALUE", /(.+)=(.+)/, "gazebo model option") { |p, k, v| @opts[:go][k] = v }

  op.on("--gazebo_model NAME")
  op.on("--imu_rate IMU_RATE", Integer)
  op.on("--hil_gps", "turn on hil_gps mode")
  op.on("--plugin_lists PATH", "path to mavros pluginlists.yaml")
  op.on("--logging", "turn on logging")
  op.on("--debug", "debug mode")
  op.on("--nomavros", "without mavros")
  op.on("--workspace PATH", "path to workspace")
  op.on("--gazebo PATH", "path to gazebo resources")
  op.on("--catkin_ws PATH", "path to catkin workspace")

  op.on("--ports_base PORT", Integer)
  op.on("--ports_step STEP", Integer)
  for k,v in @opts
    if k.start_with?('pd_')
      op.on("--#{k} PORT_DELTA", Integer, "port delta from base port")
      @port_names<<k.to_s.delete_prefix('pd_').to_sym
    end
  end
  
  op.on("--udp_sitl", "SITL with udp exchange")
  op.on("--distance DISTANCE", Integer, "distance between models")
  op.on("--optical_flow", "turn on optical flow")
  op.on("--firmware PATH", "path to firmware folder")
  op.on("--sitl_gazebo PATH", "path to sitl_gazebo folder")
  op.on("--build_label NAME", "build label")
  op.on("--single", "single mavros node")
  op.on("--hitl", "HITL mode")
  op.on("--onboard_streams PATH", "path to mavlink onboard streams file (type and rate whitespace separated on each line)")

  op.on("--restart", "soft restart")

  op.on("-h", "--help", "help and show defaults") do
    puts op
    puts "\nDefault options: #{@opts}"

    @opts[:help] = true
  end
end.parse!(into: @opts)
@opts[:gazebo_world] = ARGV[0]

if @opts[:hitl]
    @opts[:go][:serialEnabled] = 1
    @opts[:go][:hil_mode] = 1  
end

#relative paths
@rels = {
  firmware: "../../",
  sitl_gazebo: "../sitl_gazebo.m",

  firmware_etc: "ROMFS/px4fmu_common",
  firmware_init_rc: "init.d-posix/rcS",
  firmware_bin: "build/px4_sitl_#{@opts[:build_label]}/bin/px4",

  gazebo_world: "worlds/empty.world",
  gazebo_model: "models/#{@opts[:gazebo_model]}/#{@opts[:gazebo_model]}.sdf",

  workspace_world: "ws.world",
  workspace_firmware: "fw"
}

#script dir
@abs = {
  home: __dir__
}

#expand and check paths
expand_and_check()

if @opts[:help]
  puts "\nDefault absolute paths: #{@abs}"
  puts
  exit
end

init_contents()

#init
cd @abs[:home]

if @opts[:restart]
  puts "restarting ..."
  system("./kill_px4.sh")
else
  system("./kill_sitl.sh")
end
sleep 1

#start
start_firmware()
start_gazebo()
start_mavros()

#!/usr/bin/ruby

require 'optparse'
require 'fileutils'

include FileUtils

def xspawn(term_name, cmd, debug = false, env = {})
  term = debug ? "xterm -T #{term_name} -hold -e" : ""
  pid = spawn(env, "#{term} #{cmd}", [:out, :err]=>"/dev/null")
  Process.detach(pid)
end

def create_fcu_files()
  for sym in [:firmware_rc, :firmware_vars, :firmware_params, :firmware_mavlink]
    cp @abs[sym], '.'
  end

  File.open(@rels[:firmware_vars], 'a') do |out|
    out.puts "PX4_SIM_MODEL=#{@opts[:firmware_model] || @opts[:gazebo_model]}"
    out.puts "PX4_ESTIMATOR=#{@opts[:firmware_estimator]}"

    out.puts "base_port=$((#{@opts[:ports_base]}+inst*#{@opts[:ports_step]}))"
    out.puts "simulator_opts=\"-#{@opts[:udp_sitl] ? 'u' : 'c'} $((base_port+#{@opts[:pd_sim]}))\""

    out.puts "udp_gcs_port_local=$((base_port+#{@opts[:pd_gcs]}))"
    out.puts "udp_offboard_port_local=$((base_port+#{@opts[:pd_offb]}))"
    out.puts "udp_onboard_payload_port_local=$((base_port+#{@opts[:pd_payl]}))"

    out.puts "udp_gcs_port_remote=$((base_port+#{@opts[:pd_gcs_out]}))"
    out.puts "udp_offboard_port_remote=$((base_port+#{@opts[:pd_offb_out]}))"
    out.puts "udp_onboard_payload_port_remote=$((base_port+#{@opts[:pd_payl_out]}))"
  end

  File.open(@rels[:firmware_params], 'a') do |out|
    out.puts "param set SDLOG_MODE -1" unless @opts[:logging]
    #TODO
    out.puts "param set MAV_USEHILGPS 1" if @opts[:hil_gps]
  end

end

def check_expanded_path(file_name, dir = nil, msg = nil)
  p = File.expand_path(file_name, dir)

  unless File.exist?(p)
    puts "#{p} does not exist" + (msg ? ", #{msg}" : "")
    exit
  end

  p
end

def expand_firmware_files()
  for sym in [:firmware_build, :firmware_etc]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:firmware])
  end

  for sym in [:firmware_bin, :firmware_sg_build]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:firmware_build])
  end

  def_firmware_initd = File.expand_path(@rels[:firmware_initd], @abs[:firmware_etc])

  for sym in [:firmware_rc, :firmware_vars, :firmware_params, :firmware_mavlink]
    #set by user
    if @abs[:firmware_initd]
      p = File.expand_path(@rels[sym], @abs[:firmware_initd])
      @abs[sym] = p if File.exist?(p)
    end

    #default
    @abs[sym] = check_expanded_path(@rels[sym], def_firmware_initd) unless @abs[sym]
  end
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
  for sym in [:firmware, :sitl_gazebo, :plugin_lists, :world_sdf, :firmware_initd]
    @abs[sym] = check_expanded_path(@opts[sym]) if @opts[sym]
  end

  #set defaults if not set
  for sym in [:firmware]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:home]) unless @abs[sym]
  end

  for sym in [:sitl_gazebo]
    @abs[sym] = check_expanded_path(@rels[sym], @abs[:firmware]) unless @abs[sym]
  end

  #check firmware files
  expand_firmware_files()

  #gazebo resources
  for sym in [:world_sdf, :model_sdf]
    check_gazebo_resource(sym)
  end

  #workspace
  for sym in [:workspace_firmware, :workspace_model_sdf]
    @abs[sym] = File.expand_path(@rels[sym], @abs[:workspace])
  end

end

def load_contents()
  #contents
  @contents = {}
  for sym in [:world_sdf, :model_sdf]
    @contents[sym] = File.read(@abs[sym]) if @abs[sym]
  end
end

def iterate_instances
  ports = {}

  @opts[:n].times { |m_index|
    m_num = m_index+1
    model_name="#{@opts[:gazebo_model]}#{m_num}"

    p = @opts[:ports_base] + m_index*@opts[:ports_step]

    for n in @port_names
      ports[n] = p + @opts["pd_#{n}".to_sym]
    end

    yield m_index, m_num, model_name, ports
  }
end

def start_firmware()
  fw_name = @rels[:firmware_bin].split('/').last

  mkdir_p @abs[:workspace_firmware]
  cd(@abs[:workspace_firmware]) {
    create_fcu_files()

    iterate_instances { |m_index, m_num, model_name, ports|
      #firmware instance
      mkdir_p model_name
      cd(model_name) {
        #run
        xspawn("#{fw_name}-#{m_num}", "#{@abs[:firmware_bin]} #{@opts[:debug] ? '' : '-d'} -i #{m_index} -s ../#{@rels[:firmware_rc]} #{@abs[:firmware_etc]}", @opts[:debug], {'PATH'=> ENV['PATH'] + ':../'})
      }
    }
  }
end

def generate_model(tags_values, split = false)
  out = ""
  if split
    split_arr = []
  end

  @contents[:model_sdf].each_line {|l|
    found = false
    tags_values.each { |k, v|
      if l =~ /.*<#{k}>.*\n/
        if split
          split_arr << out
          out = ""
        end

        s = "      <#{k}>#{v}</#{k}>\n"
        if split
          split_arr << s
        else
          out << s
        end

        found = true
        break
      end
    }

    unless found
      out << l
    end
  }

  if split
    split_arr << out
    return split_arr
  end

  out
end

def start_gazebo()
  env = {
    'GAZEBO_PLUGIN_PATH'=> 'build',
    'GAZEBO_MODEL_PATH'=> 'models'
  }

  env.each { |k,v|
    env[k] = ''
    for sym in [:gazebo, :sitl_gazebo]
      p = @abs[sym] + '/' + v
      if sym == :sitl_gazebo and k == 'GAZEBO_PLUGIN_PATH' and @opts[:sitl_gazebo] == nil
        p = @abs[:firmware_sg_build]
      end

      env[k] += p + ':'
    end
  }
  cmd = @abs[:home] + "/gz_env.sh "

  #paths
  mkdir_p @abs[:workspace]

  @contents[:model_sdf] = generate_model(@opts[:go]) unless @opts[:go].empty?
  port_param = @opts[:udp_sitl] ? 'mavlink_udp_port' : 'mavlink_tcp_port'

  parts = generate_model({port_param => ''}, true) #three parts: part before, tag line and part after

  xspawn("gazebo", cmd + "gazebo --verbose #{@abs[:world_sdf]}", @opts[:debug], env)
  sleep 2

  iterate_instances { |m_index, m_num, model_name, ports|
    x = m_index*@opts[:distance]

    File.open(@abs[:workspace_model_sdf], 'w') do |out|
      parts.each_with_index do |p, i|
        if i == 1 #tag line with port_param
          p = "      <#{port_param}>#{@opts[:hitl] ? ports[:offb] : ports[:sim]}</#{port_param}>\n"
        end
        out << p
      end
    end

    system(env, cmd + "gz model --verbose -m #{model_name} -f #{@abs[:workspace_model_sdf]} -x #{x}", @opts[:debug] ? {} : {[:out, :err]=>"/dev/null"})
  }

  sleep 3
end

def start_mavros()
  cd("mavros") {
    iterate_instances { |m_index, m_num, model_name, ports|
      args={
        num: m_num,
        fcu_url: "udp://127.0.0.1:#{ports[:offb_out]}@127.0.0.1:#{ports[:offb]}",
        gcs_inport: ports[:gcs_mavros]
      }
      args[:plugin_lists] = @abs[:plugin_lists] if @abs[:plugin_lists]

      launch = "px4_#{@opts[:single] ? 'single' : 'num'}.launch"
      args.each { |k, v| launch<<" #{k}:=#{v}" }

      xspawn("mavros-#{m_num}", "./roslaunch.sh #{@abs[:catkin_ws]} #{launch}", @opts[:debug])
      if m_num == 1
        sleep 3
      end
    }
  }
end

####################################

#default options
@opts = {
  n: 1,
  firmware_estimator: "ekf2",
  gazebo_model: "iris",
  go: {},

  ports_base: 15010,
  ports_step: 10,
  pd_gcs: 0,
  pd_gcs_out: 5,
  pd_offb: 1,
  pd_offb_out: 6,
  pd_payl: 2,
  pd_payl_out: 7,
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
  op.on("-g PARAM=VALUE", /(.+)=(.+)/, "gazebo model option") { |p, k, v| @opts[:go][k] = v }

  op.on("--firmware_estimator NAME")
  op.on("--firmware_model NAME")
  op.on("--gazebo_model NAME")
  #TODO
  op.on("--imu_rate IMU_RATE", Integer)
  #TODO
  op.on("--hil_gps", "turn on hil_gps mode")

  op.on("--plugin_lists PATH", "path to mavros pluginlists.yaml")
  op.on("--logging", "turn on logging")
  op.on("--debug", "debug mode")
  op.on("--nomavros", "without mavros")


  op.on("--firmware_initd PATH", "path to dir with user defined firmware files ")
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
  op.on("--firmware PATH", "path to firmware folder")
  op.on("--sitl_gazebo PATH", "path to sitl_gazebo folder")
  op.on("--build_label NAME", "build label")
  op.on("--single", "single mavros node")
  op.on("--hitl", "HITL mode")

  op.on("-h", "--help", "help and show defaults") do
    puts op
    puts "\nDefault options: #{@opts}"

    @opts[:help] = true
  end
end.parse!(into: @opts)
@opts[:world_sdf] = ARGV[0]

if @opts[:hitl]
    @opts[:go][:serialEnabled] = 1
    @opts[:go][:hil_mode] = 1
end

#relative paths
@rels = {
  firmware: "../../",
    sitl_gazebo: "Tools/sitl_gazebo",
    firmware_build: "build/px4_sitl_#{@opts[:build_label]}",
      firmware_bin: "bin/px4",
      firmware_sg_build: "build_gazebo",
    firmware_etc: "ROMFS/px4fmu_common",
      firmware_initd: "init.d-posix",
        firmware_rc: "rcS",
        firmware_vars: "px4-vars.sh",
        firmware_params: "px4-params.sh",
        firmware_mavlink: "px4-mavlink.sh",

  world_sdf: "worlds/empty.world",
  model_sdf: "models/#{@opts[:gazebo_model]}/#{@opts[:gazebo_model]}.sdf",

  workspace_model_sdf: "ws.sdf",
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

load_contents()

#init
cd @abs[:home]

system("./stop.sh")
sleep 1

#start
start_gazebo()
start_firmware() unless @opts[:hitl]
start_mavros() unless @opts[:nomavros]

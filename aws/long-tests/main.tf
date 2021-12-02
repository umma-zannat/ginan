data "template_file" "myuserdata" {
  template = "${file("${path.cwd}/long-test.tpl")}"
  vars = {
    docker_image = "${var.docker_image}"
    instance_name = "${var.instance_name}"
    ginan_root = "/ginan"
  }
}


resource "aws_instance" "my-instance" {
  ami = "ami-04bc6fccb9f7d9c20"
  instance_type = "m5.4xlarge"
  key_name = "gnss_analysis_sydney"
  iam_instance_profile = "ACS_Longtests_Role"
  user_data = data.template_file.myuserdata.rendered
  instance_initiated_shutdown_behavior = "terminate"
  #spot_price = "0.5"
  tags = {
    Name = var.instance_name
    CostCentre = "ACS"
  }
  root_block_device {
    volume_size           = 500
    volume_type           = "gp2"
  }
}

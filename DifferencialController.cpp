
#include "DifferencialController.h"

DifferencialController::DifferencialController()
{
  Kp = 0.4;
  Ki = 0.6;
  Kd = 0.00;
  
  lastError = 0;
  lastErrorIntegration = 0;

  lastError1 = 0;
  lastErrorIntegration1 = 0;
  lastV = 0;
}

void DifferencialController::reset()
{
  lastError = 0;
  lastErrorIntegration = 0;

  lastError1 = 0;
  lastErrorIntegration1 = 0;
  lastV = 0;
  
}


void DifferencialController::execute(Robot *robot, Input *input, Output *output, double dt)
{

    double sv = input->v;
    double sw = input->w;

    Vel vel = robot->uni_to_diff_oneside(sv, sw); //uni_to_diff_v(sv, sw);
    if( abs(sv) <= 0.001 )
    {
      // if( abs(vel.vel_l ) < 1.2 * robot->min_vel )
      // {
      //   if( vel.vel_l < 0 )
      //   {
      //     output->vel_l = -1.2 * robot->min_vel;
      //     output->vel_r = 1.2 * robot->min_vel;
      //   }
      //   else
      //   {
      //     output->vel_l = 1.2 * robot->min_vel;
      //     output->vel_r = -1.2 * robot->min_vel;
          
      //   }
        
      // } 
      // else
      // {
      //     output->vel_l = vel.vel_l;
      //     output->vel_r = vel.vel_r;
      // }

      double dif = robot->vel_l + robot->vel_r;  //控制左右一致的转速
      if( dif * vel.vel_l > 0 ) //1. vel_l < 0 dif <0 vel_r = vel_r - dif; 2. vel_l > 0 dif > 0 vel_r - dif
      {
        vel.vel_r = vel.vel_r - 2*dif;
      }
      else // 1. vel_l < 0 dif > 0 vel_l - dif 2. vel_l > 0 dif < 0 vel_l - dif
      {
        vel.vel_l = vel.vel_l - 2*dif;
      }
      
      output->vel_l = vel.vel_l;
      output->vel_r = vel.vel_r;
      lastError = 0;
      lastErrorIntegration = 0;
      return;
    }


  if( abs(sw) > 0.2 ) //拐弯，不做速度控制
  {
    output->vel_l = vel.vel_l;
    output->vel_r = vel.vel_r;
    // lastError = 0;
    // lastErrorIntegration = 0;
  }
  else
  {
      double e, ei,ed;
      e = input->v - robot->velocity;
      ei = lastErrorIntegration + e* dt;
      ed = (e-lastError)/dt;

      sv = Kp * e + Ki*ei + Kd*ed;
      lastErrorIntegration = ei;
      lastError = e;
      vel = robot->uni_to_diff_oneside(sv, sw); //uni_to_diff_v(sv, sw);
      output->vel_l = vel.vel_l;
      output->vel_r = vel.vel_r;   
  }
  


  // Serial.print( robot->velocity);
  // Serial.print(',');
  // Serial.print(input->v);
  // Serial.print(',');
  // Serial.print(sv);
  // Serial.print(',');
  // Serial.print(sw);
  // Serial.print(',');
  // Serial.print(output->vel_l);
  // Serial.print(',');
  // Serial.println(output->vel_r);


return;


}



// void DifferencialController::execute(Robot *robot, Input *input, Output *output, double dt)
// {

//     double sv = input->v;
//     double sw = input->w;

//     // Serial.print(sv);
//     // Serial.print(',');
//     // Serial.print(sw);
//     // Serial.print(',');

//     double max_w = robot->max_w;

// 		if (sw > max_w)   //限制拐弯时的转速
// 		  sw = max_w;  
// 		else if (sw < -max_w)
// 		  sw = -max_w;

//     // if( sv != 0 )  //拐弯减速
//     // {
//     //   double av = abs(sv);
//     //   av = (0.08-av)/max_w * abs(sw) + av;
//     //   if( av <= 0 )
//     //     av = 0.01;

//     //   if( sv < 0  )
//     //     av = -av;
      
//     //   sv = av;
//     // }

//   Vel vel = robot->uni_to_diff(sv, sw);


//   if( sv == 0 && lastV != 0 )
//   {
//     lastError = 0;
//     lastErrorIntegration = 0;
//     lastError1 = 0;
//     lastErrorIntegration1 = 0;
//     if( Ki != 0) //直接初始化转圈？？？
//     {
//       lastErrorIntegration = vel.vel_l / Ki;
//       lastErrorIntegration1 = vel.vel_r /Ki;
//     }
//     output->vel_l = vel.vel_l;
//     output->vel_r = vel.vel_r;
//     lastV = sv;
//     // Serial.print(output->vel_l);
//     // Serial.print(',');
//     // Serial.println(output->vel_r);
//     return;
//   }

//   double e1, e2, ei1, ei2, ed1,ed2;
  
//   e1 = vel.vel_l - robot->vel_l;
//   e2 = vel.vel_r - robot->vel_r;

//   ei1 = lastErrorIntegration + e1 * dt;
//   ei2 = lastErrorIntegration1 + e2 * dt;
  
//   if( abs(ei1)*Ki > robot->max_vel  )
// 	  ei1 = ei1/3; 
//   if( abs(ei2)*Ki > robot->max_vel  )
// 	  ei2 = ei2/3; 

//  ed1 = (e1-lastError)/dt;
//  ed2 = (e2-lastError1)/dt;

//   double vel_l = Kp * e1 + Ki * ei1 + Kd * ed1;
//   double vel_r = Kp * e2 + Ki * ei2 + Kd * ed2;

// //限定拐弯时，不出现左右反转
//  if( sv != 0 && vel.vel_l *vel.vel_r < 0)
//  {
//     if( abs(vel.vel_l) > abs(vel.vel_r ) )
//     {
//       vel.vel_r = 0;
//       e2 = 0;
//       ei2 = 0;
//       ed2 = 0;
//       vel_r = 0;
//     }
//     else
//     {
//       vel.vel_l = 0;
//       e1 = 0;
//       ei1 = 0;
//       ed1 = 0;
//       vel_l = 0;
//     }
//   } 

//   lastError = e1;
//   lastError1 = e2;


//   // Serial.print(e1, 3);
//   // Serial.print(',');
//   // Serial.print(vel_l, 3);
//   // Serial.print(',');
//   vel_l = robot->normalizeVel(vel.vel_l, vel_l );
//   vel_r = robot->normalizeVel(vel.vel_r, vel_r );
//   if( sv == 0 ) //转圈时，限定vel;
//   {
//     if( abs(vel_l) > abs(vel.vel_l ))
//       vel_l = vel.vel_l;
//     if( abs(vel_r) > abs(vel.vel_r ))
//       vel_r = vel.vel_r;
//   }
//   // Serial.println(vel_l, 3);
//   lastErrorIntegration = ei1;
//   lastErrorIntegration1 = ei2;
//   output->vel_l = vel_l;
//   output->vel_r = vel_r;
  
//   Serial.print( robot->velocity);
//   Serial.print(',');
//   Serial.print(robot->w);
//     Serial.print(',');

//     Serial.print(output->vel_l);
//     Serial.print(',');
//     Serial.println(output->vel_r);

//   lastV = sv;
//   return;
// }


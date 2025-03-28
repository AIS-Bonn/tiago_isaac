# def f1(x, a, K, B):
#     x = x.view(-1)
#     a = a.view(-1)
#     K = K.view(-1)
#     B = B.view(-1)
    
#     C2 = (3*B + 2*K*a)/a**2
#     C3 = (-2*B - K*a)/a**3

#     return C3 * x**3 + C2 * x**2

# def f2(x, K, B):
#     return K * x + B

# def f3(x, a, b, c, L, K, B):
#     x = x.view(-1)
#     a = a.view(-1)
#     K = K.view(-1)
#     B = B.view(-1)
    
#     D0 = (3*B*b*c**2 - B*c**3 + 2*K*b**2*c**2 + L*b**3 - 3*L*b**2*c)/(b**3 - 3*b**2*c + 3*b*c**2 - c**3)
#     D1 = (-6*B*b*c - 4*K*b**2*c - K*b*c**2 - K*c**3 + 6*L*b*c)/(b**3 - 3*b**2*c + 3*b*c**2 - c**3)
#     D2 = (3*B*b + 3*B*c + 2*K*b**2 + 2*K*b*c + 2*K*c**2 - 3*L*b - 3*L*c)/(b**3 - 3*b**2*c + 3*b*c**2 - c**3)
#     D3 = (-2*B - K*b - K*c + 2*L)/(b**3 - 3*b**2*c + 3*b*c**2 - c**3)

#     return D3 * x**3 + D2 * x**2 + D1 * x + D0

# def f(x, L, a, b, c, K, B):
#     x = x.view(-1)
#     L = L.view(-1)
#     a = a.view(-1)
#     b = b.view(-1)
#     c = c.view(-1)
#     B = B.view(-1)
#     return torch.where(x < a, f1(x, a, K, B), torch.where(x < b, f2(x, K, B), f3(x, a, b, c, L, K, B)))





# class S_Curve_net(nn.Module):
#     def __init__(self):
#         super(S_Curve_net, self).__init__()

#         # Reintroduce hidden layers for nonlinear regression
#         self.fc1 = nn.Linear(1, 10)  
#         self.fc2 = nn.Linear(10, 10)
#         self.fc3 = nn.Linear(10, 5)  

#         # Weight Initialization
#         self._initialize_weights()

#     def _initialize_weights(self):
#         for m in self.modules():
#             if isinstance(m, nn.Linear):
#                 nn.init.kaiming_uniform_(m.weight, nonlinearity='relu')
#                 if m.bias is not None:
#                     nn.init.constant_(m.bias, 0)



#     def forward(self, x, L):
#         L = L.view(-1, 1)
#         x = x.view(-1, 1)

#         # Pass K through the fully connected layers with ReLU activations
#         out = torch.relu(self.fc1(L))
#         out = torch.relu(self.fc2(out))
#         out = self.fc3(out)

#         a = torch.nn.functional.sigmoid(out[:, 0]) * 0.5 # ensure a < 0.5
#         b = out[:, 1] + a # ensure b > a and b-a < 5
#         c = torch.nn.functional.sigmoid(out[:, 2]) * 0.5 + b # ensure c > b and c - b < 0.5
#         B = torch.nn.functional.softplus(out[:, 3])
#         K = torch.nn.functional.softplus(out[:, 4])
#         # Compute the Gompertz function with the predicted parameters
#         out = f(x=x, L=L, a=a, b=b, c=c, K=K, B=B)
#         return out, a, b, c, B, K
    
# def f1_double_prime(x, a, K, B):
#     a = a.view(-1)
#     K = K.view(-1)
#     B = B.view(-1)
    
#     C3 = (-2*B - K*a) / a**3
#     C2 = (3*B + 2*K*a) / a**2
    
#     # Second derivative of f1(x)
#     return 6 * C3 * x + 2 * C2

# def f3_double_prime(x, a, b, c, L, K, B):
#     b = b.view(-1)
#     c = c.view(-1)
#     K = K.view(-1)
#     B = B.view(-1)
#     L = L.view(-1)
    
#     D3 = (-2*B - K*b - K*c + 2*L) / (b**3 - 3*b**2*c + 3*b*c**2 - c**3)
#     D2 = (3*B*b + 3*B*c + 2*K*b**2 + 2*K*b*c + 2*K*c**2 - 3*L*b - 3*L*c) / (b**3 - 3*b**2*c + 3*b*c**2 - c**3)
    
#     # Second derivative of f3(x)
#     return 6 * D3 * x + 2 * D2

# # def loss_function(input, output, target, a, b, c, L, K, B, lambda_1=1.0, lambda_2=1.0):
# #     mse_loss = nn.MSELoss()(output, target)  # Main task loss (e.g., MSE)

# #     # Convexity penalty for f1 (sample points in [0, a])
# #     f1_double_prime_vals = f1_double_prime(input, a, K, B)
# #     convexity_penalty = torch.relu(-f1_double_prime_vals).mean()  # Penalize negative second derivative

# #     # Concavity penalty for f3 (sample points in [b, c])
# #     f3_double_prime_vals = f3_double_prime(input, a, b, c, L, K, B)
# #     concavity_penalty = torch.relu(f3_double_prime_vals).mean()  # Penalize positive second derivative

# #     # Total loss with penalties
# #     total_loss = mse_loss + lambda_1 * convexity_penalty + lambda_2 * concavity_penalty
# #     return total_loss

# def loss_function(input, output, target, a, b, c, L, K, B, lambda_1=1.0, lambda_2=1.0):
#     mse_loss = nn.MSELoss()(output, target)  # Main task loss (e.g., MSE)

#     # Apply convexity penalty only in the domain [0, a]
#     mask_f1 = (input >= 0) & (input <= a)  # Mask for the range [0, a]
#     f1_double_prime_vals = f1_double_prime(input, a, K, B) * mask_f1.float()
#     convexity_penalty = torch.relu(-f1_double_prime_vals).mean()  # Penalize negative second derivative

#     # Apply concavity penalty only in the domain [b, c]
#     mask_f3 = (input >= b) & (input <= c)  # Mask for the range [b, c]
#     f3_double_prime_vals = f3_double_prime(input, a, b, c, L, K, B) * mask_f3.float()
#     concavity_penalty = torch.relu(f3_double_prime_vals).mean()  # Penalize positive second derivative

#     # Total loss with penalties
#     total_loss = mse_loss + lambda_1 * convexity_penalty + lambda_2 * concavity_penalty
#     return total_loss
